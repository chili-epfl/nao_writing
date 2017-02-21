#!/usr/bin/env python
"""
Listens for a trajectory to write and sends it to the nao via naoqi SDK.

Requires a running robot/simulation with ALNetwork proxies.

"""
from naoqi import ALModule, ALBroker, ALProxy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from copy import deepcopy
import rospy
import tf
import motion
import math



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


            timeList.append(time)
           
            # ------ add time --------
            if points:
                # if more than 3mm between points, smeans letter separation, give more time to reach this point
                if gapBetweenPoints([points[-1][1], points[-1][2]], [target_robot.pose.position.y, target_robot.pose.position.z]) > 0.03:
                    time = time + 0.1 #traj.poses[i+1].header.stamp - trajp.header.stamp
                else:
                    time = time + 0.036
            else:
                time = time + 0.036

            # ------- add position ----------
            points.append([0, target_robot.pose.position.y, target_robot.pose.position.z, roll, 0, 0])
            
        
        #wait until time instructed to start executing
        rospy.sleep(traj.header.stamp-rospy.Time.now())
        rospy.loginfo("executing rest of traj at "+str(rospy.Time.now()))
        startTime = rospy.Time.now()

        # send position to proxy
        motionProxy.positionInterpolations(effectorList, space, scaleAndFlipPoints(points), axisMaskList, timeList, True)

        rospy.loginfo("Time taken for rest of trajectory: "+str((rospy.Time.now()-startTime).to_sec()))
    else:
        rospy.loginfo("Got traj but not allowed to execute it because I've fallen")


def scaleAndFlipPoints(matrix):

    # windows where the robot can move its arms
    rangeOfPossibleY = [-0.2, 0]
    rangeOfPossibleZ = [0.0, 0.15]
    posRefRobot = 0.3



    vectY = [m[1] for m in matrix]
    vectZ = [m[2] for m in matrix]
    meanY = sum(vectY)/float(len(vectY))
    meanZ = sum(vectZ)/float(len(vectZ))
    
    # flip Y
    vectY = [i - 2*(i - meanY) for i in vectY]

    # center y & z
    maxY = float(max(vectY))
    minZ = float(min(vectZ))

    vectY = [i - maxY for i in vectY]
    vectZ = [i - minZ for i in vectZ]

    # scale up y
    scaleFactorY = abs(max(rangeOfPossibleY) - min(rangeOfPossibleY))/abs(float(max(vectY)) - float(min(vectY)))
    vectY = [i*scaleFactorY for i in vectY]

    # scale up Z
    scaleFactorZ = abs(max(rangeOfPossibleZ) - min(rangeOfPossibleZ))/abs(float(max(vectZ)) - float(min(vectZ)))
    vectZ = [i*scaleFactorZ+posRefRobot for i in vectZ]

    # compute X, the length of the arm should be approximately constant depending on z, y -> need to find x
    z0 = 0.45
    y0 = -0.12
    r = 0.2
    vectX = [math.sqrt(-math.pow((z-z0), 2) - math.pow((vectY[i]-y0), 2) + r*r) for i, z in enumerate(vectZ)]
   



    points = []
    for i, y in enumerate(vectY):
        points.append([vectX[i], y, vectZ[i], matrix[0][3], 0, 0])

    return points

def gapBetweenPoints(pointa, pointb):

    return math.sqrt(math.pow((pointa[0] - pointb[0]), 2) + math.pow((pointa[1] - pointb[1]), 2))

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
    trackerProxy = ALProxy("ALTracker", NAO_IP, port)

    fallResponder = FallResponder("fallResponder",motionProxy,memoryProxy);

    pub_traj = rospy.Subscriber(TRAJ_TOPIC, Path, on_traj)

    pub = rospy.Publisher('roger', String, queue_size=10)
    
    motionProxy.wbEnableEffectorControl(effector,False) #if robot has fallen it will have a hard time getting up if the effector is still trying to be kept in a particular position
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
