# Mission 1
The robot project will be developed using a number of design goals to build up the system to be able to accomplish the missions.

Note: In this article I'm going to explain the basic functionality for my ROS nodes not how you write ROS nodes or compile them, there is plenty of resources out there on the internet for that. To fully understand the node functionality please read the documentation for each package available in its GitHub repository.
## Design Goal 1
To be able to look around using the camera, search for faces, attempt to identify any seen and display a message for any identified. This will require:
- Control of head/camera using RC servos
- Access to Pi Camera
- Facial detections and recognition
### Pan & Tilt
To control the head/camera we need a pan and tilt device which will require two RC servos. I expect that the project will also in the future require a second device for a LIDAR (Light Detection and Ranging) sensor. We therefore straight away require four PWM outputs, not to mention any required for motors in the future. The Raspberry Pi only has one hardware PWM, although software PWMs could be used, so I'm going to pass the control of the servos off to a second board.

I could have used a purpose built board like one available from PiBorg, the [UltraBorg](https://www.piborg.org/sensors-1136/ultraborg "UltraBorg"). Using this board you can connect up to four servos and four HC-SR04 ultrasonic devices to the Raspberry Pi using an I2C bus.

However, since I have a number of Arduino Nano's available, from a previous project, I'm going to make use of one of those. To control the Arduino I'm going to use an already available ROS package that includes a node for communicating with the Arduino over the serial port and an Arduino library for use in the Arduino sketch. This is our first example of being able to take advantage of a ROS package written by some else and made available from the ROS Wiki website [rosserial_arduino](http://wiki.ros.org/rosserial_arduino "rosserial_arduino")

In order to be able to make use of this package I need to install the package on the ROS workstation, install the library in the Arduino IDE environment. This will also include rebuilding this Arduino library if we use any user defined ROS messages (which we will). How to do this and much more is covered on the Wiki in [rosserial arduino tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials "Tutorials")

To be able to make use of the rosserial_arduino package I will need to write code (a sketch) for the Arduino that controls the servos and a ROS node for the functionality of a Pan/Tilt device. The rosserial node will sit between these two items. Also as indicated above I'll use a user defined ROS message to communicate between them.

The code for the Arduino sketch will accept a ROS message which will contain an index value, indicating which servo is to be moved, and a value for the angle that the servo should be moved to. As I'll have to recompile the Arduino library to include my user defined message the first ROS package to write is the one containing this message. This package also contains a second message which will be used by the pan and tilt node. This package is available in the GitHub Repository https://github.com/phopley/servo_msgs See the package documentation for full details.

Next I needed to write the Arduino sketch that will accept the message and move the servos. This sketch is available in a folder in this repository https://github.com/phopley/rodney/tree/master/arduino See the README.md file for sketch details.

In the sketch I map the four servo outputs to the physical pins where the servos are connected. 
``` C++
/* Define the PWM pins that the servos are connected to */
#define SERVO_0 9
#define SERVO_1 6
#define SERVO_2 5
#define SERVO_3 3
```
To test this part of the system I connected two servo to the Arduino nano as shown in the following diagrams. The nano is powered through the USB connection from the Raspberry Pi

![alt text](https://github.com/phopley/rodney/blob/master/docs/images/Nano%20prototpe_bb.png "prototpe_bb") ![alt text](https://github.com/phopley/rodney/blob/master/docs/images/Nano%20prototpe_schem.png "prototpe_schem")

Later the pan and tilt node will allow configuration to map a servo to either pan or tilt on a particular pan/tilt device. 

The final part of the pan and tilt functionality is the pan_tilt package which makes up the pan tilt node. This package is available in the GitHub Repository https://github.com/phopley/pan_tilt See the package documentation for details.
This node can control two pan/tilt devices, one is expected to move the head/camera and the other for a LIDAR.

The package includes a configuration file which maps the servos to a pan/tilt device and also position within the device, the range of a servo can be constrained and trim values for each servo included.

The package also contains a launch file to test the pan_tilt_node and the Arduino sketch. The launch file will launch the pan_tilt_node, the serial_node and remaps the pan_tilt_node/index0_position topic to be the pan_tilt_node/head_position.
``` XML
<?xml version="1.0" ?>
<launch>
  <rosparam command="load" file="$(find pan_tilt)/config/config.yaml" />
  <node pkg="pan_tilt" type="pan_tilt_node" name="pan_tilt_node" output="screen">
    <remap from="pan_tilt_node/index0_position" to="pan_tilt_node/head_position" />
  </node>
  <node pkg="rosserial_python" type="serial_node.py" name="serial_node" output="screen" args="/dev/ttyUSB0" />
</launch>
```
To run the nodes using the launch file, `cd` to the workspace where the packages were built, source the setup bash file and call roslaunch.
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch pan_tilt pan_tilt_test.launch
```
The following ROS Graph depicts the pan tilt part of the system.

![alt text](https://github.com/phopley/rodney/blob/master/docs/images/rosgraph_pantilt.png "Pan tilt graph")

With the system running as shown in the graph above change the pan tilt servo positions using rostopic and the following command
`rostopic pub -1 /pan_tilt_node/head_position servo_msgs/pan_tilt {45,45}`

This will command the pan and tilt servos of the first pan and tilt device to be both 45 degrees.

In the next part of the design goal 1 I'll look at the face recognition package.
