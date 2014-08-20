#!/usr/bin/env python

"""
Takes an SVG file containing a path as input, and publishes the corresponding 
trajectory on the 'write_traj' topic using nav_msgs/Path message. The timestamps
of the PoseStamped vector in the Path are the time from start of the point in 
the trajectory.

Requires:
    - svg2traj (which itself relies on softMotion-libs) to compute the trajectory
    OR
    - svg_subsampler to compute the trajectory (from cowriter-trajectory-generator)

"""

import logging
logger = logging.getLogger("write." + __name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

import argparse
import subprocess
import rospy

from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Empty

def get_traj(svgfile):
    logger.info("Running " + SVG_SAMPLER + "...")
    if(SVG_SAMPLER=="svg2traj"):
        p = subprocess.Popen([SVG_SAMPLER, svgfile], stdout=subprocess.PIPE, stderr = subprocess.PIPE)
    elif(SVG_SAMPLER=="svg_subsampler"):    
        p = subprocess.Popen([SVG_SAMPLER, svgfile, str(SAMPLE_DENSITY), SAMPLE_TYPE, YFLIP], stdout=subprocess.PIPE, stderr =    subprocess.PIPE)
    
    traj, errors = p.communicate()
    logger.info(errors)

    traj = traj.strip().split("\n")

    # first line of the output of svg2traj is the x,y origin of the path in meters,
    # relative to the SVG document origin
    x_orig, y_orig = [float(x) for x in traj[0].split()]

    for l in traj[1:]:
        x, y, z = l.split()
        if(SVG_SAMPLER=="svg2traj"):
            x=x_orig + float(x);
            y=y_orig + float(y);
        elif(SVG_SAMPLER=="svg_subsampler"):
            x=float(x)
            y=float(y)

        yield Vector3(x, y, 0) # stange values on Z! better set it to 0


def make_traj_msg(points):
    traj = Path()
    traj.header.frame_id = FRAME
    traj.header.stamp = rospy.Time.now()+rospy.Duration(delayBeforeExecuting);
    
    for i, p in enumerate(points):
        point = PoseStamped();
        point.pose.position = p;
        point.header.frame_id = FRAME;
        point.header.stamp = rospy.Time(t0+i*dt); #assume constant time between points for now
        traj.poses.append(point)
    return traj

if __name__ == "__main__":
    rospy.init_node("trajectory_publisher")
    FRAME = rospy.get_param('~writing_surface_frame_id','writing_surface')
    CLEAR_TOPIC = rospy.get_param('~clear_surface_topic','clear_screen')
    TRAJ_TOPIC = rospy.get_param('~trajectory_output_topic','write_traj')
    TRAJ_TOPIC_NAO = rospy.get_param('~visualization_output_topic','write_traj_downsampled')


    #SVG_SAMPLER = "svg2traj"
    SVG_SAMPLER = rospy.get_param('~svg_sampler',"svg_subsampler")
    YFLIP_BOOL = rospy.get_param('~flip_svg_y_axis',True) #yflip or no-yflip
    if(YFLIP_BOOL):
        YFLIP = 'yflip'
    else:
        YFLIP = 'no-yflip'

    # svg_subsampler parameters
    SAMPLE_DENSITY = rospy.get_param('~sample_pts_per_cm', 5) # points per cm
    #SAMPLE_TYPE = rospy.get_param('~sample_method',"homogeneous")
    SAMPLE_TYPE = "homogeneous" #'homogeneous' or 'curvature' (which has some difficulty with paths which are too straight or too curved)
    dt = rospy.get_param('~seconds_between_pts',0.1); #seconds between points in traj
    t0 = rospy.get_param('~seconds_for_first_pt',3.5); #extra time for the robot to get to the first point in traj
    timeBetweenDisplays = rospy.get_param('~seconds_between_publishing',2.5);
    delayBeforeExecuting = rospy.get_param('~seconds_before_execution',0.5);


    pub_traj_vis = rospy.Publisher(TRAJ_TOPIC, Path, queue_size=5)
    pub_traj_nao = rospy.Publisher(TRAJ_TOPIC_NAO, Path, queue_size=5)
    pub_clear = rospy.Publisher(CLEAR_TOPIC, Empty, queue_size=5)

    parser = argparse.ArgumentParser(description='Publish an SVG trajectory as a ROS Path')
    parser.add_argument('-f', '--file', action="store",
                    help='an SVG file containing a single path')

    args, unknown = parser.parse_known_args()
    raw_traj = list(get_traj(args.file))
    while not rospy.is_shutdown():
        pub_clear.publish(Empty())
        traj = make_traj_msg(raw_traj)
        pub_traj_nao.publish(traj)
        pub_traj_vis.publish(traj)
        if(timeBetweenDisplays > 0):
            rospy.sleep(traj.poses[-1].header.stamp.to_sec()+timeBetweenDisplays) #wait until have to display again
