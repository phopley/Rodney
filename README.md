# rodney
ROS package for the main control node for Rodney robot
## Description
This node is the main control node of the robot. Currently it is activated by use of a keyboard with the following key commands:
- 's' results in the robot moving its head to scan for known faces, if a face is recognised a welcome message is sent to the log system.
- 'c' results in any current scan being cancelled.
- 'x' clears the list of faces already recognised.
## Executable
rodney_node
## Subscribed topics
- keyboard/keydown of type keyboard::Key
## Action client
- face_recognition_msgs::scan_for_faces