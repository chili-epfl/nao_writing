nao_writing
===========

A set of ROS nodes which allow a robot to perform simulated handwriting through motions which are synchronised with an android tablet's display.

The `nao_trajectory_following` nodes allow for receiving a message and getting a Nao robot to trace it in the air at the appropriate TF frame representing the writing surface. 
Example usage:
`roslaunch nao_trajectory_following draw_svg_demo.launch`

The `nao_writing_android_nodes` package provides an android-based node suitable for deployment on a tablet which displays the requested message while the robot is tracing it in the air.
Example usage:
`roslaunch nao_trajectory_following message_echoer_nao.launch`
