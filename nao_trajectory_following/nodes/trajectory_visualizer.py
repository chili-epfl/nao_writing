#!/usr/bin/env python

"""
Listens on a topic for a trajectory message, and publishes the corresponding 
trajectory with markers as an animation suitable for RViz.

"""

import logging
logger = logging.getLogger("write." + __name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_msgs.msg import Empty


potentialShapesMissed = 20; #perhaps a clear request is received for shapes 
                            #displayed before the node started, so shapeCount 
                            #won't be accurate. This is the number of 
                            #(potential) shapes to delete in such a case.


def visualize_traj(points):

    traj = Marker()
    traj.header.frame_id = FRAME
    traj.header.stamp = rospy.get_rostime()
    traj.ns = "writing_traj"
    traj.action = Marker.ADD
    traj.pose.orientation.w = 1.0
    traj.type = Marker.LINE_STRIP
    traj.scale.x = 0.01 # line width
    traj.color.r = 1.0
    traj.color.b = 0.0
    traj.color.a = 1.0
    
    if(WRITE_MULTIPLE_SHAPES):
        traj.id = shapeCount;
    else:
        traj.id = 0; #overwrite any existing shapes
        traj.lifetime.secs = 1; #timeout for display
    
    traj.points = list(points)
    
    pub_markers.publish(traj)

def on_traj(requested_traj):
    global shapeCount 
    written_points = []
    
    #wait until time instructed to start executing
    rospy.sleep(requested_traj.header.stamp-rospy.Time.now());

    #wait for robot to get to starting point
    rospy.sleep(requested_traj.poses[0].header.stamp.to_sec()); 

    #add points to the display one at a time, like an animation
    for i in range(len(requested_traj.poses)-1): 
        p = requested_traj.poses[i].pose.position;
        written_points.append(p)
        visualize_traj(written_points)
        duration = requested_traj.poses[i+1].header.stamp - requested_traj.poses[i].header.stamp;
        rospy.sleep(duration); #wait until it's time to show the next point
        
    #show final point (no sleep afterwards, but it may have a "lifetime" set 
    #in visualize_traj)    
    p = requested_traj.poses[len(requested_traj.poses)-1].pose.position;
    written_points.append(p)
    visualize_traj(written_points)

    shapeCount += 1;

def on_clear(message):
    global shapeCount

    #clear each of the trajectories displayed
    for i in range(max(shapeCount,potentialShapesMissed)):
        traj = Marker()
        traj.header.frame_id = FRAME
        traj.header.stamp = rospy.get_rostime()
        traj.ns = "writing_traj"
        traj.action = Marker.DELETE
        traj.id = shapeCount;
        pub_markers.publish(traj)
    shapeCount = 0;
    
if __name__=="__main__":
    rospy.init_node("trajectory_visualizer");
    
    CLEAR_TOPIC = rospy.get_param('~clear_surface_topic','clear_screen')
    SHAPE_TOPIC = rospy.get_param('~trajectory_input_topic','write_traj')
    MARKER_TOPIC = rospy.get_param('~visualization_output_topic','visualization_markers')
    FRAME = rospy.get_param('~writing_surface_frame_id','writing_surface') 

    WRITE_MULTIPLE_SHAPES = True; #if True, modify the marker ID so as to not 
                                    #overwrite the previous shape(s)
    
    pub_markers = rospy.Publisher(MARKER_TOPIC, Marker, queue_size=5);
    
    shapeCount = 0;
    #when we get a trajectory, start publishing the animation
    traj_subscriber = rospy.Subscriber(SHAPE_TOPIC, Path, on_traj);
    #when we get a clear request, delete previously drawn shapes
    clear_subscriber = rospy.Subscriber(CLEAR_TOPIC, Empty, on_clear);
    
    rospy.spin()
