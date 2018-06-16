# Design Goals
The robot project will be developed using a number of design goals.

Note: In this article I'm going to explain functionality for my ROS nodes not how you write ROS nodes or compile them, there is plenty of resources out there on the internet for that. To fully understand the node functionality read the README.md file for each package.
## Design Goal 1
Use Case - On power up look around using the camera, search for faces, attempt to identify any seen and display a message for any identified. This will require:
- Control of head/camera using RC servos
- Access to Pi Camera
- Facial detections and recognition
### Pan & Tilt
To control the head/camera we need a pan and tilt device which will require two RC servos. I expect that the project will also in the future require a second device for a LIDAR (Light Detection and Ranging) sensor. We therefore straight away require four PWM outputs, not to mention any required for motors in the future. The Raspberry Pi only has one hardware PWM, although software PWMs could be used, so I'm going to pass the control of the servos off to a second board.

You could use a purpose built board like one available from PiBorg, the [UltraBorg](https://www.piborg.org/sensors-1136/ultraborg "UltraBorg"). Using this board you can connect up to four servos and four HC-SR04 ultrasonic devices to the Raspberry Pi using an I2C bus.

However, since I have a number of Arduino Nano's available from a previous project I'm going to make use of one of those. To attach and control the Arduino I'm also going to make use of a ROS package designed to communicate with an Arduino its USB serial port. This is our first example of being able to take advantage of a ROS package written by some else and made available from the ROS Wiki website [rosserial_arduino](http://wiki.ros.org/rosserial_arduino "rosserial_arduino")

To get the pan and tilt functionality I need to do four things:
- Install the rossserial package
- Install the rosserial libiary for the Arduino IDE
- Write the code that will run on the Arduino
- Write the ROS node containing the pan/tilt functionality

To install the ROS serial package on my develoment PC and on the Raspberry Pi `sudo apt-get install ros-kinetic-rosserial`
The user must also have permission to open he port `sudo adduser <username> dialout`. With the roscore already running the serial node can be started with the command `rosrun rosserial_python serial_node.py /dev/ttyUSB0`

In order to be able to compile an Arduino sketch to use this node you need to install the ros serial arduino package, `sudo apt-get install ros-kinetic-rosserial-arduino`. Note I'm using the Arduino IDE installed on my Linux box. The library also needs building. 

```
cd Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

The code for the Arduino sketch will accept a ROS message which will contain an index value indicating which servo is to me moved and a value for the angle that the servo should be moved to. Before we can compile the sketch you have to recompile the Arduino library to include this ROS message.

My ROS package containing this message also contain a second message which you be used by the pan tilt node. The ROS package is available in the GitHub Repository https://github.com/phopley/servo_msgs See the package documentation for details..

With the servo_msgs packages built as part of the Catkin workspace catkin_ws, rebuld the Arduino library 
```
cd ~/catkin_ws
source devel/setup.bash
cd Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

You can now compile an Arduino sketch that will use the `servo_array.msg` which is part of the servo_msgs package. The Arduino sketch is available in the GitHub Repository folder https://github.com/phopley/arduino-projects/tree/master/ServoControl4Channel See the README.md file for sketch details.

This sketch accepts a topic servo of type servo_msgs::servo_array and moves the selected servo to the given positon.

The final part of the pan and tilt functionality is the pan_tilt package which makes up the pan tilt node. This package is available in the GitHub Repository https://github.com/phopley/pan_tilt See the package documentation for details.
This node can control two pan/tilt devices, one is expected to move the head/camera and the other for a LIDAR.

The package contains a launch file to test the pan_tilt_node and the Arduino sketch. The launch file will launch the pan_tilt_node, the serial_node and remaps the pan_tilt_node/index0_position topic to be the pan_tilt_node/head_position.
```
<?xml version="1.0" ?>
<launch>
  <rosparam command="load" file="$(find pan_tilt)/config/config.yaml" />
  <node pkg="pan_tilt" type="pan_tilt_node" name="pan_tilt_node" output="screen">
    <remap from="pan_tilt_node/index0_position" to="pan_tilt_node/head_position" />
  </node>
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node" output="screen" args="/dev/ttyUSB0" />
</launch>
```
If the packages have been built in the workspace catkin_ws and the Arduino is programmed and connected, launch the nodes with
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch pan_tilt pan_tilt_test.launch
```
The following ROS Graph depicts the pan tilt part of the system.

![alt text](https://github.com/phopley/rodney/blob/master/docs/images/rosgraph_pantilt.png "Pan tilt graph")

With the system running as shown in the graph above change the pan tilt servo positions using rosstopic and the following command
`rostopic pub -1 /pan_tilt_node/head_position servo_msgs/pan_tilt {45,45}`

This will command the pan and tilt servos of the first pan and tilt device to both be 45 degrees. There are parameter server values available in the pan_tilt package to configure which servo is connected to which servo and which device and to limit the range of a servo as well as to trim the servo. See the package documentation for details.
### Head Control
TBA
### Face Recognition
TBA
### Rodney Control
