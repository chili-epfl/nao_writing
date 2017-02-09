nao_trajectory_following
========================

This package contains a set of nodes useful for getting a Nao humanoid to trace a trajectory at the same time and position as a synchronised display (e.g. the accompanying Android app).

Tested with ROS Hydro and Indigo.

Provided nodes
--------------
- `message_echoer_nao.py`: listens for a set of `nav_msgs/Path` messages on a ROS topic (such as strokes acquired from an Android app) and re-publishes them for the Nao to execute in sync with a display device (e.g. an Android app). [requires python naoqi SDK]

- `nao_write_naoqi.py`: reads a cartesian trajectory from a ROS topic and executes the corresponding joint-space trajectory on the Nao using naoqi. [requires python naoqi SDK]

- `nao_trajectory_following writing_surface_positioner.py`: displays a rectangular-shaped interactive marker for RViz that may be used to manually place the writing surface in space.

- `nao_trajectory_following publish_svg_traj.py`: takes an SVG file as input and publishes it as a `nav_msgs/Path` message on a ROS topic. [requires softMotion or [cowriter-trajectory-generator](https://github.com/chili-epfl/cowriter-trajectory-generator) for the SVG conversion]
  
- `nao_trajectory_following trajectory_visualizer.py`: listens for a `nav_msgs/Path` message on a ROS topic and publishes it as a set of markers suitable for display in RViz as an animation.
