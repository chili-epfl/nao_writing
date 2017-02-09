#!/usr/bin/env python
"""
Listens for a trajectory to write and sends it to the nao via naoqi SDK.

Requires a running robot/simulation with ALNetwork proxies.

"""
from naoqi import ALModule, ALBroker, ALProxy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from copy import deepcopy
import rospy
import tf
import motion



### trajectoire main robot
def on_traj(traj):
    rospy.loginfo("got traj at "+str(rospy.Time.now())) 
    if(hasFallen == False): #no harm in executing trajectory
        if(effector == "LArm"):
            motionProxy.openHand("LHand");
            roll = -1.7 #rotate wrist to the left (about the x axis, w.r.t. robot frame)
        else:
            motionProxy.openHand("RHand");
            roll = 1.7 #rotate wrist to the right (about the x axis, w.r.t. robot frame)

        target = PoseStamped()
        target_frame = traj.header.frame_id
        target.header.frame_id = target_frame
        
        points = []
        timeList = []
        time = 3.0
        for i, trajp in enumerate(traj.poses):
        
            trajp.pose.position.z = 0.1
            target.pose.position = deepcopy(trajp.pose.position)
            target.pose.orientation = deepcopy(trajp.pose.orientation)
            target_robot = tl.transformPose("base_footprint",target)

            
            points.append([0.12, target_robot.pose.position.y, target_robot.pose.position.z, roll, 0, 0])
            timeList.append(time)

            time = time + 0.036 #traj.poses[i+1].header.stamp - trajp.header.stamp
        

        #wait until time instructed to start executing
        rospy.sleep(traj.header.stamp-rospy.Time.now())
        rospy.loginfo("executing rest of traj at "+str(rospy.Time.now()))
        startTime = rospy.Time.now()

        motionProxy.positionInterpolations(effectorList, space, ScaleAndFlipPoints(points, 1.5), axisMaskList, timeList, True)
        rospy.loginfo("Time taken for rest of trajectory: "+str((rospy.Time.now()-startTime).to_sec()))
    else:
        rospy.loginfo("Got traj but not allowed to execute it because I've fallen")

def ScaleAndFlipPoints(matrix, scaleFactor = 2):

    vectY = [m[1] for m in matrix]
    vectZ = [m[2] for m in matrix]

    meanY = sum(vectY)/float(len(vectY))
    meanZ = sum(vectZ)/float(len(vectZ))
    
    # flip Y
    vectY = [i - 2*(i - meanY) for i in vectY]
    # scale up vector Y and be carefull to set positions that NAO can reach
    maxY = float(max(vectY))
    vectY = [i - maxY - 0.05 for i in vectY]
    vectY = [(i-meanY)*scaleFactor + meanY for i in vectY]

    # scale up Z
    vectZ = [(i-meanZ)*scaleFactor + meanZ for i in vectZ]


    points = []
    for i, y in enumerate(vectY):
        points.append([0.15, y, vectZ[i], matrix[0][3], 0, 0])

    return points


class FallResponder(ALModule):
  """ Module to react to robotHasFallen events """
  
  def __init__(self, name, motionProxy, memoryProxy):
      ALModule.__init__(self, name)
      self.motionProxy = motionProxy;
      memoryProxy.subscribeToEvent("robotHasFallen",name,self.has_fallen.__name__);
      rospy.loginfo("Subscribed to robotHasFallen event");
  def has_fallen(self, *_args):
      global hasFallen
      hasFallen = True;
      self.motionProxy.killAll();
      rospy.loginfo("Stopped task");
   



if __name__ == "__main__":
    rospy.init_node("nao_writer");
    
    TRAJ_TOPIC = rospy.get_param('~trajectory_nao_input_topic','/write_traj_nao')    
    NAO_IP = rospy.get_param('~nao_ip','127.0.0.1'); #default behaviour is 
                                        #to connect to simulator locally
    NAO_HANDEDNESS = rospy.get_param('~nao_handedness','right')
    if(NAO_HANDEDNESS.lower()=='right'):
        effector   = "RArm"
    elif(NAO_HANDEDNESS.lower()=='left'):
        effector = "LArm"
    else: 
        rospy.logerr('error in handedness param')



    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    port = 9559;
    myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
        "0.0.0.0",   # listen to anyone
        0,           # find a free port and use it
        NAO_IP,      # parent broker IP
        port)        # parent broker port
    hasFallen = False;
    motionProxy = ALProxy("ALMotion", NAO_IP, port);
    memoryProxy = ALProxy("ALMemory", NAO_IP, port);
    postureProxy = ALProxy("ALRobotPosture", NAO_IP, port)
    fallResponder = FallResponder("fallResponder",motionProxy,memoryProxy);
    pub_traj = rospy.Subscriber(TRAJ_TOPIC, Path, on_traj)
    
    motionProxy.wbEnableEffectorControl(effector,False); #if robot has fallen it will have a hard time getting up if the effector is still trying to be kept in a particular position
    postureProxy.goToPosture("StandInit", 0.2)

    tl = tf.TransformListener()

    motionProxy.wbEnableEffectorControl(effector,True);
    rospy.sleep(2)


    space = motion.FRAME_ROBOT
    effectorList = ["RArm"]
    axisMaskList = [motion.AXIS_MASK_X+motion.AXIS_MASK_Y+motion.AXIS_MASK_Z+motion.AXIS_MASK_WX]
    isAbsolute = True
    
    rospy.spin()
    myBroker.shutdown()
