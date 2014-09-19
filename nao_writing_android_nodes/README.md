##message_echoer Android app
The `message_echoer` app is a user interface for getting the Nao to "write": it captures messages for the Nao to trace and re-displays them in synch with the robot as it traces them.

###Dependencies
A pre-compiled apk file is provided, but for compilation from source the [ROS for Android](http://wiki.ros.org/android) library is necessary on the compiling computer. (The rosjava.rosinstall and android_core.rosinstall files at https://github.com/rosjava/rosjava/tree/master may be used for installation with [these instructions](http://wiki.ros.org/rosjava/Tutorials/hydro/Source%20Installation).)

###Provded classes 
- `MainActivity` manages the screen which will be shown to the user. The primary function of the `MainActivity` class is to initialise the UI and necessary ROS nodes which run on the tablet, and to assign the necessary connections between the other classes (e.g. callback functions). At start-up it will launch a ROS `MasterChooser` UI to get the URI of the ROS master and NTP server to be used. Afterwards, it will show the main view as defined in res/layout/main.xml which contains a `SystemDrawingViewNode` for displaying requested system shapes, and a `UserDrawingView` for displaying/collecting messages to be written. Buttons for clearing user-drawn shapes and for sending the user-drawn shapes are also displayed. 

- `SystemDrawingViewNode` is a ROS node which receives a message of a shape to be displayed and displays it on the screen as an animation. Several shapes may be displayed (using a `LayerDrawable`), until a clear_screen message is received, at which point all shapes will be removed.

- `InteractionManagerNode` is a ROS node which listens for user input such as user demonstrations, clicks and other gestures and publishes the event information on the relevant topics. 

- `UserDrawingView` is a class for capturing user drawings, either drawn with a fingertip or stylus. It displays the drawing as it occurs.

- `DisplayMethods.java` contains some utility methods/classes related to displaying (e.g. `AnimationDrawableWithEndCallback` for notifying when shape display is finished).


Note: Shapes are published as `nav_msgs/Path` messages, with the xy origin at the bottom left. If the message is a display request (as opposed to a user-drawn shape), the time stamp in the header indicates the desired start time of the message, and the time stamp of the individual points indicates the time, relative to the start time, that the point should be displayed. If the sequence ID for a point is 1, that point is considered as bein on a "pen-up" segment, and the line to that point will not be displayed.

Note: By default, the system waits until the time requested in the message header before animating the shape (to synchronise with other devices such as a robot writing), but this may be modified by specifying the `wait_to_sync_traj` ROS param value. 
