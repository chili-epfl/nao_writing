#!/usr/bin/env python

#
# ROS node to provide access to the camera by wrapping NaoQI access (may not
# be the most efficient way...)
#
# Copyright 2012 Daniel Maier, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy
from sensor_msgs.msg import Image, CameraInfo
from naoqi_driver.naoqi_node import NaoqiNode
import camera_info_manager


from naoqi import ALProxy
from vision_definitions import *

PUBLISH_STATUS_TOPIC = 'camera_publishing_status';
from std_msgs.msg import Bool
publishStatus = False;
class NaoCam (NaoqiNode):
    def __init__(self):
        NaoNode.__init__(self)
        rospy.init_node('nao_camera')

        self.camProxy = self.getProxy("ALVideoDevice")
        if self.camProxy is None:
            exit(1)

        # check if camera_switch is admissible
        self.camera_switch = rospy.get_param('~camera_switch', 0)
        if self.camera_switch == 0:
            self.frame_id = "/CameraTop_frame"
        elif self.camera_switch == 1:
            self.frame_id = "/CameraBottom_frame"
        else:
            rospy.logerr('Invalid camera_switch. Must be 0 or 1')
            exit(1)
        rospy.loginfo('using camera: %d', self.camera_switch)

        # Parameters for ALVideoDevice
        self.resolution = rospy.get_param('~resolution', kVGA)
        self.colorSpace = rospy.get_param('~color_space', kBGRColorSpace)
        self.fps  = rospy.get_param('~fps', 30)

        # ROS publishers
        self.pub_img_ = rospy.Publisher('~image_raw', Image, queue_size=10)
        self.pub_info_ = rospy.Publisher('~camera_info', CameraInfo, queue_size=10) 

        # initialize camera info manager
        self.cname = "nao_camera" # maybe it's a good idea to add _top and _bottom here ...
        self.cim = camera_info_manager.CameraInfoManager(cname=self.cname)
        self.calibration_file_bottom = rospy.get_param('~calibration_file_bottom', None)
        self.calibration_file_top = rospy.get_param('~calibration_file_top', None)
        if self.camera_switch == 0:
            calibration_file = self.calibration_file_top
        else:
            calibration_file = self.calibration_file_bottom

        if not self.cim.setURL( calibration_file ):
            rospy.logerr('malformed URL for calibration file')
            sys.exit(1)
        else:
            try:
                self.cim.loadCameraInfo()
            except IOExcept:
                rospy.logerr('Could not read from existing calibration file')
                exit(1)

        if self.cim.isCalibrated():
            rospy.loginfo('Successfully loaded camera info')
        else:
            rospy.logerr('There was a problem loading the calibration file. Check the URL!')
            exit(1)

        # subscription to camProxy
        self.nameId = ''
        self.subscribe()
        
        # subscription to requests to start/stop publishing
        self.sub_publish = rospy.Subscriber(PUBLISH_STATUS_TOPIC,Bool,self.setPublishingStatus)
        
        # capture a dummy image
        self.dummyImage = self.camProxy.getImageRemote(self.nameId)
        

    def subscribe(self):
        # unsubscribe for all zombie subscribers
        self.camProxy.unsubscribeAllInstances("rospy_gvm")

        # subscribe
        self.nameId = self.camProxy.subscribe("rospy_gvm", self.resolution, self.colorSpace, self.fps)
        #print "subscriber name is ", self.nameId

        # set params
        rospy.sleep(1)
        self.camProxy.setParam(kCameraSelectID, self.camera_switch)
        self.camProxy.setResolution(self.nameId, self.resolution)

    def main_loop(self):
        img = Image()
        r = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            if(publishStatus == True):
                image = self.camProxy.getImageRemote(self.nameId)
                stampAL = image[4] + image[5]*1e-6
                #print image[5],  stampAL, "%lf"%(stampAL)
                img.header.stamp = rospy.Time(stampAL)
                img.header.frame_id = self.frame_id
                img.height = image[1]
                img.width = image[0]
                nbLayers = image[2]
                if image[3] == kYUVColorSpace:
                    encoding = "mono8"
                elif image[3] == kRGBColorSpace:
                    encoding = "rgb8"
                elif image[3] == kBGRColorSpace:
                    encoding = "bgr8"
                elif image[3] == kYUV422ColorSpace:
                    encoding = "yuv422" # this works only in ROS groovy and later
                else:
                    rospy.logerror("Received unknown encoding: {0}".format(image[3]))

                img.encoding = encoding
                img.step = img.width * nbLayers
                img.data = image[6]
                infomsg = self.cim.getCameraInfo()
                infomsg.header = img.header
                self.pub_info_.publish(infomsg)
                self.pub_img_.publish(img)
            r.sleep() #todo have different rates because if only publish at 1Hz still want to 'wake up' faster


        self.camProxy.unsubscribe(self.nameId)

    '''
    #this is not reliable. it seems that the unsubscribe only works (sometimes) when the node dies.
    class subscriptionListener(rospy.SubscribeListener):
        def peer_subscribe(self, topic_name, topic_publish, peer_publish):
            print("got subscription")
            subscribersConnected = True;
            
        def peer_unsubscribe(self, topic_name, num_peers):
            global subscribersConnected
            print("got unsub")
            if num_peers == 0:
                print('no one left');
                subscribersConnected = False;
    '''
    def setPublishingStatus(self,message):
        global publishStatus
        publishStatus = message.data
        print(message);
        if(publishStatus == False):
            print('Turning publishing off')
            img = Image()
            for i in range(5):
                image = self.dummyImage
                #waste an image
                stampAL = image[4] + image[5]*1e-6
                #print image[5],  stampAL, "%lf"%(stampAL)
                img.header.stamp = rospy.Time(stampAL)
                img.header.frame_id = self.frame_id
                img.height = image[1]
                img.width = image[0]
                nbLayers = image[2]
                if image[3] == kYUVColorSpace:
                    encoding = "mono8"
                elif image[3] == kRGBColorSpace:
                    encoding = "rgb8"
                elif image[3] == kBGRColorSpace:
                    encoding = "bgr8"
                elif image[3] == kYUV422ColorSpace:
                    encoding = "yuv422" # this works only in ROS groovy and later
                else:
                    rospy.logerror("Received unknown encoding: {0}".format(image[3]))

                img.encoding = encoding
                img.step = img.width * nbLayers
                img.data = image[6]
                infomsg = self.cim.getCameraInfo()
                infomsg.header = img.header
                self.pub_info_.publish(infomsg)
                self.pub_img_.publish(img)
        else:
            print('Turning publishing on')
            self.subscribe()
            img = Image()
            for i in range(5):
                image = self.dummyImage
                #waste an image
                stampAL = image[4] + image[5]*1e-6
                #print image[5],  stampAL, "%lf"%(stampAL)
                img.header.stamp = rospy.Time(stampAL)
                img.header.frame_id = self.frame_id
                img.height = image[1]
                img.width = image[0]
                nbLayers = image[2]
                if image[3] == kYUVColorSpace:
                    encoding = "mono8"
                elif image[3] == kRGBColorSpace:
                    encoding = "rgb8"
                elif image[3] == kBGRColorSpace:
                    encoding = "bgr8"
                elif image[3] == kYUV422ColorSpace:
                    encoding = "yuv422" # this works only in ROS groovy and later
                else:
                    rospy.logerror("Received unknown encoding: {0}".format(image[3]))

                img.encoding = encoding
                img.step = img.width * nbLayers
                img.data = image[6]
                infomsg = self.cim.getCameraInfo()
                infomsg.header = img.header
                self.pub_info_.publish(infomsg)
                self.pub_img_.publish(img)
                
if __name__ == "__main__":
    try:
        naocam = NaoCam()
        naocam.main_loop()
    except RuntimeError, e:
        rospy.logerr('Something went wrong: %s' % (e.strerrror) )
    rospy.loginfo('Camera stopped')
