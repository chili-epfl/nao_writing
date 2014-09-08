#!/usr/bin/env python

"""
A ROS node to receive messages as a series of strokes and publish them again as requests suitable for simulated robotic writing.
"""


import numpy

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Empty

rospy.init_node("message_echoer_nao");


#Nao parameters
NAO_IP = rospy.get_param('~nao_ip','127.0.0.1'); #default behaviour is to connect to simulator locally
naoWriting = rospy.get_param('~nao_writing',True); #whether or not the robot should move its arms
naoConnected = rospy.get_param('~use_robot_in_interaction',True); #whether or not the robot is being used for the interaction (looking, etc.)



NAO_HANDEDNESS = rospy.get_param('~nao_handedness','right')
if(NAO_HANDEDNESS.lower()=='right'):
    effector = "RArm"
elif(NAO_HANDEDNESS.lower()=='left'):
    effector = "LArm"
else: 
    rospy.logerr('error in handedness param')
    
#trajectory publishing parameters
factor = 12;
FRAME = rospy.get_param('~writing_surface_frame_id','writing_surface') ;  #Frame ID to publish points in
SHAPE_TOPIC = rospy.get_param('~trajectory_output_topic','/write_traj'); #Name of topic to publish shapes to

if(naoWriting):
    timeForFirstStroke = 3;#Time allowed for the first point in traj (seconds)
    timeBetweenStrokes = .7;
    dt = 0.35               #Seconds between points in traj
    delayBeforeExecuting = 3;#How far in future to request the traj be executed (to account for transmission delays and preparedness)
elif(naoConnected):
    timeForFirstStroke = 0.05;
    timeBetweenStrokes = 0.05;
    dt = 0.1;
    delayBeforeExecuting = 3.5;
else:
    timeForFirstStroke = 0.05;
    timeBetweenStrokes = 0.05;
    dt = 0.1;
    delayBeforeExecuting = 3.5;
numDesiredShapePoints = 15.0;#Number of points to downsample the length of shapes to 

#tablet parameters
tabletConnected = True;      #If true, will wait for shape_finished notification before proceeding to the next shape (rather than a fixed delay)
minTimeBetweenTouches = 0.1  #Seconds allowed between touches for the second one to be considered

CLEAR_SURFACE_TOPIC = rospy.get_param('~clear_writing_surface_topic','clear_screen');
SHAPE_FINISHED_TOPIC = rospy.get_param('~shape_writing_finished_topic','shape_finished');
USER_DRAWN_SHAPES_TOPIC = rospy.get_param('~user_drawn_shapes_topic','user_drawn_shapes');

pub_traj = rospy.Publisher(SHAPE_TOPIC, Path, queue_size=5);
pub_clear = rospy.Publisher(CLEAR_SURFACE_TOPIC, Empty, queue_size=5);



from numpy import mean;
def downsample_1d(myarr,factor,estimator=mean):
    """
   FROM http://code.google.com/p/agpy/source/browse/trunk/AG_image_tools/downsample.py?r=452
    Downsample a 1D array by averaging over *factor* pixels.
    Crops right side if the shape is not a multiple of factor.

    This code is pure numpy and should be fast.

    keywords:
        estimator - default to mean.  You can downsample by summing or
            something else if you want a different estimator
            (e.g., downsampling error: you want to sum & divide by sqrt(n))
    """
   #xs = myarr.shape
    xs = len(myarr);
    factor = int(factor);
    crarr = myarr[:xs-(xs % factor)]
    dsarr = estimator( numpy.concatenate([[crarr[i::factor]
        for i in range(factor)] ]),axis=0)
    return dsarr



strokes=[];
### ------------------------------------------------------ MESSAGE MAKER
def read_traj_msg(message):
    global firstStrokeOfMessage,strokes
    
    rospy.loginfo('Got stroke to write with '+str(len(message.poses))+' points');
    if(len(message.poses)==0):
        publishMessage(strokes); 
        strokes = [];
    else:
        x_shape = [];
        y_shape = [];
        for poseStamped in message.poses:
            x_shape.append(poseStamped.pose.position.x);
            y_shape.append(-poseStamped.pose.position.y);
            
        numPointsInShape = len(x_shape); 

        shape = [];
        shape[0:numPointsInShape] = x_shape;
        shape[numPointsInShape:] = y_shape;
        
        shape = numpy.reshape(shape, (-1, 1)); #explicitly make it 2D array with only one column
        strokes.append(shape);

def onShapeFinished(message):
    global shapeFinished;
    shapeFinished = True;
    
shapeFinished = False    
def publishMessage(strokes):
    global shapeFinished
    rospy.loginfo('Got message to write with '+str(len(strokes))+' strokes');
    for i in range(len(strokes)):
        stroke = strokes[i];
        if(i==0):
            firstStrokeOfMessage = True;
        else:
            firstStrokeOfMessage = False;
        publishShape(stroke,firstStrokeOfMessage);
        
        #listen for notification that the letter is finished
        shape_finished_subscriber = rospy.Subscriber(SHAPE_FINISHED_TOPIC, String, onShapeFinished);
        rospy.loginfo('Waiting for shape to finish');
        while(not shapeFinished):
            rospy.sleep(0.1);
        shape_finished_subscriber.unregister();
        shapeFinished = False;
        rospy.loginfo('Shape finished.');
    
    
def make_traj_msg(shape, headerString, firstStrokeOfMessage):      
    
    traj = Path();
    traj.header.frame_id = FRAME;
    traj.header.stamp = rospy.Time.now()+rospy.Duration(delayBeforeExecuting);
    numPointsInShape = len(shape)/2;   
    
    x_shape = shape[0:numPointsInShape];
    y_shape = shape[numPointsInShape:];

    x_shape = downsample_1d(x_shape,factor);
    y_shape = downsample_1d(y_shape,factor);

    numPointsInShape = len(x_shape);
    
    for i in range(numPointsInShape):
        point = PoseStamped();
        point.pose.position.x = x_shape[i];
        point.pose.position.y = -y_shape[i];
        
        #point.pose.position.x+= + shapeCentre[0];
        #point.pose.position.y+= + shapeCentre[1];
        
        point.header.frame_id = FRAME;
        if(firstStrokeOfMessage):
            point.header.stamp = rospy.Time(timeForFirstStroke+i*dt);
        else:
            point.header.stamp = rospy.Time(timeBetweenStrokes+i*dt); 

        traj.poses.append(point);

    return traj
    
###

def lookAndAskForFeedback(toSay):
    if(naoWriting):
        #put arm down
        motionProxy.angleInterpolationWithSpeed(effector,armJoints_standInit, 0.8)
    
    if(effector=="RArm"):   #person will be on our right
        motionProxy.setAngles(["HeadYaw", "HeadPitch"],[-0.9639739513397217, 0.08125996589660645],0.8);
    else:                   #person will be on our left
        motionProxy.setAngles(["HeadYaw", "HeadPitch"],[0.9639739513397217, 0.08125996589660645],0.8);

def onShapeFinished(message):
    global shapeFinished;
    shapeFinished = True;
        
### ------------------------------------------------------ PUBLISH SHAPE        
def publishShape(shape,firstStrokeOfMessage):
    trajStartPosition = Point();

    headerString = '_';
    traj = make_traj_msg(shape, headerString, firstStrokeOfMessage);
    if(firstStrokeOfMessage):
        if(naoConnected):
            trajStartPosition = traj.poses[0].pose.position;
            #nao.look_at([trajStartPosition.x,trajStartPosition.y,trajStartPosition.z,FRAME]); #look at shape     
    rospy.loginfo('publishing stroke');   
    pub_traj.publish(traj);   
    
    return trajStartPosition
        
### --------------------------------------------------------------- MAIN
firstStrokeOfMessage = True;
shapeFinished = False;
if __name__ == "__main__":
    #listen for user-drawn shapes
    shape_subscriber = rospy.Subscriber(USER_DRAWN_SHAPES_TOPIC, Path, read_traj_msg); 
    
    if(naoConnected):
        from naoqi import ALBroker, ALProxy
        #start speech (ROS isn't working..)
        port = 9559;
        myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
            "0.0.0.0",   # listen to anyone
            0,           # find a free port and use it
            NAO_IP,      # parent broker IP
            port)        # parent broker port
        textToSpeech = ALProxy("ALTextToSpeech", NAO_IP, port)   
        textToSpeech.setLanguage('English')
        motionProxy = ALProxy("ALMotion", NAO_IP, port);
        postureProxy = ALProxy("ALRobotPosture", NAO_IP, port)
        
        if(naoWriting):
            postureProxy.goToPosture("StandInit",0.8)
            armJoints_standInit = motionProxy.getAngles(effector,True);
            motionProxy.wbEnableEffectorControl(effector, True); #turn whole body motion control on
        
    rospy.loginfo('Waiting for message to write');

    rospy.spin();
