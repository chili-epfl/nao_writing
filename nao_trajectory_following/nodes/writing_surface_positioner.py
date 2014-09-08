#!/usr/bin/env python
'''
Publish marker and frame representing a writing surface, either with an 
interactive marker or wherever a fiducial marker has been detected (e.g. a 
chilitag).
'''

import roslib; roslib.load_manifest("interactive_markers")
import rospy
from geometry_msgs.msg import Pose
import tf

from visualization_msgs.msg import Marker, InteractiveMarkerControl
tf_broadcaster = tf.TransformBroadcaster()

from interactive_markers.interactive_marker_server import *

server = None
def processFeedback(feedback):
    global frame_pose
    p = feedback.pose.position
    o = feedback.pose.orientation
    frame_pose = feedback.pose

def writing_surface():

    surface = Marker()
    surface.pose.orientation.w = 1.0
    surface.pose.position.z = -.0005
    surface.pose.position.x = SURFACE_WIDTH/2
    surface.pose.position.y = SURFACE_HEIGHT/2
    surface.id = 99
    surface.type = Marker.CUBE
    surface.scale.x = SURFACE_WIDTH
    surface.scale.y = SURFACE_HEIGHT
    surface.scale.z = 0.0005
    surface.color.b = 1.0
    surface.color.g = 1.0
    surface.color.r = 1.0
    surface.color.a = 1.0

    return surface


def make6DofMarker( fixed = False ):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/map"
    int_marker.scale = 0.05

    int_marker.pose.orientation.x = frame_pose.orientation.x
    int_marker.pose.orientation.y = frame_pose.orientation.y
    int_marker.pose.orientation.z = frame_pose.orientation.z
    int_marker.pose.orientation.w = frame_pose.orientation.w
    int_marker.pose.position.x = frame_pose.position.x
    int_marker.pose.position.y = frame_pose.position.y
    int_marker.pose.position.z = frame_pose.position.z

    int_marker.name = FRAME_ID
    int_marker.description = "Place the writing surface"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = SURFACE_WIDTH
    box_marker.scale.y = SURFACE_HEIGHT
    box_marker.scale.z = 0.001
    box_marker.color.r = 1.0
    box_marker.color.g = 1.0
    box_marker.color.b = 1.0
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( writing_surface() )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

if __name__=="__main__":
    rospy.init_node("writing_surface_positioner")
        
    #method used for positioning the writing surface frame
    POSITIONING_METHOD = rospy.get_param('~positioning_method',
                                            'interactive_marker')
    
    #name of frame to publish as writing surface origin (at bottom left, with 
    #x horizontal and y vertical)
    FRAME_ID = rospy.get_param('~writing_surface_frame_id','writing_surface') 

    #size of marker to be displayed (default values for the galaxy note 10.1 
    #in landscape orientation
    SURFACE_WIDTH = rospy.get_param('~surface_width',0.217) 
    SURFACE_HEIGHT = rospy.get_param('~surface_height',0.136)
        
    
    if(POSITIONING_METHOD.lower() == "fiducial_marker_detection"):
        TAG_FRAME = rospy.get_param('~tag_frame_id','tag_1') #name of frame to 
                                                #detect writing surface with 
        ROTATE_TAG_FRAME = rospy.get_param('~rotate_tag_frame',True) #chilitag 
        #frame has y horizontal and x vertical (graphics coordinate system) and
        #needs to be changed to 'robotics' coordinate system
        
        tf_listener = tf.TransformListener(True, rospy.Duration(10))
        rospy.sleep(.5)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
        #tf_broadcaster.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"v4l_frame","gaze") #manually "attach" the webcam to the robot's frame (for testing)
            try:
                tf_listener.waitForTransform("map", TAG_FRAME, rospy.Time.now(), rospy.Duration(5.0))
                t = tf_listener.getLatestCommonTime("map", TAG_FRAME)
                (trans,rot) = tf_listener.lookupTransform("map", TAG_FRAME, t)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
                
            #rotate coordinate system of tag to match the desired one for the tablet
            surfacePose = PoseStamped()
            surfacePose.header.frame_id = TAG_FRAME
            
            if(ROTATE_TAG_FRAME):
                #convert 'x up, y to the right' of chilitag from to 'y up, x to the right' for the writing surface
                orientation = tf.transformations.quaternion_from_euler(3.14159,0,3.14159/2)

            surfacePose.pose.orientation.x=orientation[0]
            surfacePose.pose.orientation.y=orientation[1]
            surfacePose.pose.orientation.z=orientation[2]
            surfacePose.pose.orientation.w=orientation[3]
            surfacePose = tf_listener.transformPose("map",surfacePose)
            o=surfacePose.pose.orientation
            #publish writing surface's frame w.r.t. map
            tf_broadcaster.sendTransform(trans,(o.x,o.y,o.z,o.w),rospy.Time.now(),FRAME_ID,"map")
            pub_markers.publish(writing_surface()) #show writing surface

        rate.sleep()
    
    elif(POSITIONING_METHOD.lower() == "interactive_marker"):
        NAO_HANDEDNESS = rospy.get_param('~nao_handedness','right')
        
        #assign default values of pose
        frame_pose = Pose()
        #use values from 'rosrun tf tf_echo map writing_surface' with  
        #interactive marker in desired position
        if(NAO_HANDEDNESS.lower() == 'right'):
            frame_pose.orientation.x = -0.4
            frame_pose.orientation.y = 0.5
            frame_pose.orientation.z = 0.6
            frame_pose.orientation.w = -0.5
            frame_pose.position.x = 0.225
            frame_pose.position.y = 0.02
            frame_pose.position.z = 0.27
        elif(NAO_HANDEDNESS.lower() == 'left'):
            frame_pose.orientation.x = 0.4
            frame_pose.orientation.y = -0.4
            frame_pose.orientation.z = -0.5
            frame_pose.orientation.w = 0.6
            frame_pose.position.x = 0.16
            frame_pose.position.y = 0.032+SURFACE_WIDTH
            frame_pose.position.z = 0.27
        else:
            rospy.logerr('error in handedness input')
        
        server = InteractiveMarkerServer("writing_surface_placer")

        make6DofMarker(fixed = True)

        # 'commit' changes and send to all clients
        server.applyChanges()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if frame_pose:
                p = frame_pose.position
                o = frame_pose.orientation
                tf_broadcaster.sendTransform((p.x, p.y, p.z),
                                (o.x, o.y, o.z, o.w),
                                rospy.Time.now(),
                                FRAME_ID,
                                "map")
            rate.sleep()
