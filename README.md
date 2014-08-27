nao_writing: Simulated handwriting for the Nao humanoid
=======================================================

A set of ROS nodes which allow a robot to perform simulated handwriting through motions which are synchronised with an android tablet's display.

![Photo of Nao writing on tablet](https://github.com/chili-epfl/nao_writing/raw/master/doc/naoWritingWithTablet.png)

##Usage without a tablet connected

The `nao_trajectory_following` nodes allow for receiving a message and getting a Nao robot to trace it in the air at the appropriate TF frame representing the writing surface. 

####With a webots simulated Nao running:

```
roslaunch nao_trajectory_following draw_svg_demo.launch
```

####With a [ROS-enabled Nao](https://github.com/ros-nao/nao_robot):
On the robot:

```
export ROS_MASTER_URI=http://(computer's IP):11311
roslaunch nao_bringup nao.launch
```

On the computer acting as the ROS master:

```
roslaunch nao_trajectory_following draw_svg_demo.launch use_sim_nao:=false nao_ip:=(nao's IP)
```

##Usage with a tablet connected

The `nao_writing_android_nodes` package provides an android-based node suitable for deployment on a tablet which displays the requested message while the robot is tracing it in the air. It can also capture user-drawn messages for the robot to draw.

####With a webots simulated Nao running

(With the `message_echoer` app deployed on the tablet)

```
roslaunch nao_trajectory_following message_echoer_nao.launch
```

####With a [ROS-enabled Nao](https://github.com/ros-nao/nao_robot)
(With the `message_echoer` app deployed on the tablet)

On the robot:

```
export ROS_MASTER_URI=http://(computer's IP):11311
roslaunch nao_bringup nao.launch
```

On the computer acting as the ROS master:

```
roslaunch nao_trajectory_following draw_svg_demo.launch use_sim_nao:=false nao_ip:=(nao's IP)
```

![Photo of expected result with simulated nao](https://github.com/chili-epfl/nao_writing/raw/master/doc/nao_writing_demo.JPG)

*An example result of the robot (simulated with Webots) following the user-drawn trajectory in sync with the tablet's display.*

Dependencies
------------
- [naoqi SDK](https://community.aldebaran.com/resources/archives/) for motion control, 
- [nao_robot](https://github.com/ros-nao/nao_robot) for TF frames of the robot, 
- [ROS for Android](https://github.com/rosjava/rosjava_core) for the tablet app,
- (optional) softMotion or [cowriter-trajectory-generator](https://github.com/chili-epfl/cowriter-trajectory-generator) for SVG demo,
- (optional) [ros_markers](https://github.com/chili-epfl/ros_markers) for detecting the tablet's location.

For more information see the readmes in internal directories and the parameters which are available for specification in the launch files.
