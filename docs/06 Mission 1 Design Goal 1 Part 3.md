## Head Control
This package is available in the GitHub Repository https://github.com/phopley/head_control. See the package documentation for details. This package contains a ROS node which synchronises the head movement, using the pan_tilt_node, with the face recognition functionality.

It can be commanded using a ROS action to move the head/camera to a number of positions and requests a face recognition scan at each position. The action will return any faces detected in the feedback /results part of the action. This is comparable to a human looking around to identify people by moving the neck.

The ROS action, scan_for_faces, is part of the face_recognition_msgs package, which is available in the GitHub Repository https://github.com/phopley/face_recognition_msgs. The goal part of this action is empty so the action server starts the action on receipt of the empty goal message. The feedback part of the messages contains a percentage complete figure and a list of faces recognised (IDs and names) in the latest scan position. The result part of the action contains a list of faces recognised in the final scan position.

The package includes a configuration file to set parameters in the server parameter.
```
head:
  position:
    pan: 90
    tilt: 45
  step:
    pan: 10
    tilt: 10
```
The pan/tilt position value is the position the head will return to once all the scans have been completed. The pan/tilt step is the amount of movement in degrees between each scan.
## Rodney
This package is available in the GitHub Repository https://github.com/phopley/rodney. See the package documentation for details. This package will form the overall control node of Rodney robot. Currently for this design goal it keeps track of people recognised logging a welcome message the first time they are seen. The keyboard can be used to control the face recognition scans if the keyboard node is running. The keyboard package repository is https://github.com/lrse/ros-keyboard. 

The key commands are:
- 's' Start the action to look around for known faces
- 'c' Cancel any action current running and return the head to the default position
- 'x' Clear the list of faces seen

The rodney package includes a number of scripts to help build the nodes required for the design goal. The script `./clonepackages` clones the required packages from GitHub and the script `./createlinks` creates symbolic links from the git repository to the catkin workspace.
The package also includes a launch file to load configuration files and launch the nodes required to test the design goal.
``` XML
<?xml version="1.0" ?>
<launch>
  <rosparam command="load" file="$(find pan_tilt)/config/config.yaml" />
  <rosparam command="load" file="$(find face_recognition)/config/config.yaml" />
  <rosparam command="load" file="$(find head_control)/config/config.yaml" />

  <include file="$(find raspicam_node)/launch/camerav2_1280x960.launch" />
  <node name="republish" type="republish" pkg="image_transport" output="screen" args="compressed in:=/raspicam_node/image/ raw out:=/camera/image/raw" />

  <node pkg="pan_tilt" type="pan_tilt_node" name="pan_tilt_node" output="screen">
    <remap from="pan_tilt_node/index0_position" to="pan_tilt_node/head_position" />
  </node>
  <node pkg="face_recognition" type="face_recognition_node.py" name="face_recognition_node" output="screen" />
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node" output="screen" args="/dev/ttyUSB0" />
  <node pkg="head_control" type="head_control_node" name="head_control_node" output="screen" />
  <node pkg="rodney" type="rodney_node" name="rodney_node" output="screen" />
</launch>
```
To run the nodes using the launch file, `cd` to the workspace, source the setup bash file and call roslaunch.
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch rodney rodney.launch
```
The following ROS Graph depicts the system for design goal 1 of the "Take a message to..." mission.

![alt text](https://github.com/phopley/rodney/blob/master/docs/images/rosgraph_m1dg1.png "Mission1, Design goal 1")

With the system running as shown in the graph above on a Raspberry Pi do the following on a PC running Linux having setup the master (see link on running a distributed system below):
- Start the keyboard node, `rosrun keyboard keyboard`
- Start an instance of console with  `rqt_console`
- With the keyboard node having the focus press the 's' key.

Any faces recognised will be logged to the console.

![alt text](https://github.com/phopley/rodney/blob/master/docs/images/Screenshot%20from%202018-06-23%2018-49-47.png "Console output")

Notes:
- For a tutorial on running a distributed system see the [following tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines "Multiple Machines") 
- The package versions used to test design goal 1 here were:
  - rodney v???
  - face_recognition v???
  - face_recognition_msgs v???
  - head_control v???
  - pan_tilt v???
  - servo_msgs v???
  - ?arduino code? v???
  - ros-keyboard v???
  - raspicam v0.2.2 is this the correct version?
  - rosserial_python v???
  - image_transport v???
