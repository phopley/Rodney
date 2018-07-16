# Mission 1, Design Goal 1
To accomplish this design goal we will need to

* Control the head/camera using RC servos for pan/tilt movement
* Access images from the Raspberry Pi Camera
* Detect and recognize faces
* Control the order of these actions

To control the head/camera we need a pan and tilt device which will require two RC servos. I expect that the project will also in the future require a second pan/tilt device for a LIDAR (Light Detection and Ranging) sensor. We therefore straight away require four PWM outputs to control the servos, not to mention any required for motors in the future. The Raspberry Pi only has one hardware PWM and although we could make use of software PWMs, I'm goind to avoid that overhead by passing control of the servos off to a second board.

We could use a purpose built board like the one available from [PiBorg, the UltraBorg](https://www.piborg.org/sensors-1136/ultraborg "PiBorg, the UltraBorg"). Using this board you can connect up to four servos and four HC-SR04 ultrasonic devices to the Raspberry Pi using an I2C bus. However, since I have a number of Arduino Nano's available from a previous project, I'm going to make use of one of those.

This is also going to be our first of many examples in taking advantage of work already carried out by the ROS community, allowing us to concentrate on the robot application. To attach to the ROS like node which will be running on the Arduino, we are going to use a package that includes a node for communicating with the Arduino over the serial port and an Arduino library for use in the Arduino sketch. This package documentation is available on the ROS Wiki website [rosserial_arduino](http://wiki.ros.org/rosserial_arduino "[rosserial_arduino").

To make use of this package we need to install it on the ROS target and install the library in the Arduino IDE environment. We will also need to rebuild the Arduino library if we use any user defined ROS messages (which we will). How to do this and much more is covered on [rosserial arduino tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials "rosserial arduino tutorials").

To control the position of each servo making up the pan/tilt devices we will write a ROS package whose node will take pan/tilt demand messages and turn them into individual position messages that will be sent to the Arduino. The first message will identify which pan/tilt device is to be moved and the required position for the pan servo and the tilt servo. The second message, sent to the Arduino, will contain an index value, indicating which of the four servos is to be moved, and a value for the angle to move that servo to. Splitting this functionality down means that the Arduino sketch only needs to understand about servos and not pan/tilt device, it therefore could be reused for other servo applications. BTW in the world of Arduino programming the code running on the Arduino is known as a sketch and I’ll continue to use that term here. If you are unfamiliar with Arduino programming there are plenty of articles about Arduinos on the Code Project site.

We could include the definition of our user defined messages in the pan tilt package but again in the interest of reuse we will create a separate package for the message definitions.

So to complete the pan/tilt functionality we are going to write two ROS packages and a ROS style Arduino sketch.

We will call the first of these packages *servo_msgs* and it will define our messages. When built it will produce .h files for use by C++ code and will automatically create Python scripts. We will also recompile the Arduino library to produce .h files that will be used by our sketch.

The files that make up this first package are available in the *servo_message folder*. The root of this folder contains a readme file documenting the package and two files that are required to always be present in a ROS package. These are the CmakeList.txt and the package.xml files, information about these files can be found in the tutorial on [creating ROS packages](http://wiki.ros.org/ROS/Tutorials/CreatingPackage "creating ROS packages").

The msg folder within the package contains the definition files for our messages. The first of these is *servo_array.msg*

``` Python
# index references the servo that the angle is for, e.g. 0, 1, 2 or 3
# angle is the angle to set the servo to
uint8 index
uint16 angle
```

You can think of this as a C like structure. This is the message which will be sent as a ROS topic to the Arduino. The message contains two elements, __index__ indicates which of the servos is to be moved and __angle__ is the angle to move the servo to in degrees.

The second message type will be sent as two topics to the pan and tilt node and is the *pan_tilt.msg*. One topic will relate to the head/camera pan tilt device and the other will be used later for the LIDAR pan tilt device.

``` Python
int16 pan   # the angle for the pan servo
int16 tilt  # the angle for the tilt servo
```

Here the __pan__ element is the angle in degrees for the pan servo and the __tilt__ element is the angle in degrees for the tilt servo.

That completes our first simple ROS package, our second package is the *pan_tilt* package. This package is available in the *pan_tilt* folder and contains executable code which will form the *pan_tilt_node*.

The root folder of this package again includes a documentation file and the CmakeList.txt and package.xml files. This package includes a number of sub folders which I'll briefly describe. The *config* folder contains the file *config.yaml*. This file will be used by the launch file (see below) to set the given parameters in the parameter server. This will allow us to configure the system without having to recompile the code.

```
# Configuration for pan/tilt devices
# In Rodney index0 is for the head and index 1 is for the LIDAR
servo:
  index0:
    tilt_max: 100
    tilt_min: 0
    pan_servo: 0
    tilt_servo: 1
  index1:
    pan_servo: 2
    tilt_servo: 3
```

In this config file, __index0__ gives parameters for the head pan and tilt device and __index1__ for the LIDAR pan and tilt device. The ___max__ and ___min__ allow us to restrict the travel of a servo and the ___servo__ parameters identify which servo (0-3) is attached to which pan/tilt device and in which position.

The *cfg* folder contains the file *pan_tilt.cfg*. This file is used by the dynamic reconfiguration server so that we can adjust the trim of the servos on the fly. As you can see the file is actually a Python script.

``` Python
#!/usr/bin/env python
PACKAGE = "pan_tilt"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("index0_pan_trim",  int_t, 0, "Index 0 - Pan Trim",  0,  -45, 45)
gen.add("index0_tilt_trim", int_t, 0, "Index 0 - Tilt Trim", 0,  -45, 45)
gen.add("index1_pan_trim",  int_t, 0, "Index 1 - Pan Trim",  0,  -45, 45)
gen.add("index1_tilt_trim", int_t, 0, "Index 1 - Tilt Trim", 0,  -45, 45)

exit(gen.generate(PACKAGE, "pan_tilt_node", "PanTilt"))
```

For a complete understanding of the dynamic reconfiguration server refer to the ROS Wiki [section dynamic reconfiguration](http://wiki.ros.org/dynamic_reconfigure "section dynamic reconfiguration"). For now in our file you can see that we add four parameters, one for each servo and that the default value of each parameter is zero with the minimum value set to -45 and the maximum value set to 45.

The *launch* folder contains launch files which enable us to not only to load configuration files but to start all the nodes that make up a system. In our folder we have a *pan_tilt_test.launch* file which is used for testing just the pan/tilt part of the Rodney system. As you can see below this is an xml formatted file.

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

For a complete understanding of launch files refer to the ROS Wiki [section on launch files](http://wiki.ros.org/roslaunch/XML "section on launch files"). Our launch file first finds and loads our config file.

``` XML
<rosparam command="load" file="$(find pan_tilt)/config/config.yaml" />
```

The next set of tags will result in our *pan_tilt_node* being executed and remaps one of the topics so that we can easily see that it is the pan/tilt message for the head/camera. This is another little trick to help with reuse of the package. Notice also that with __output="screen"__ we will direct any logging messages to the terminal that we launched from.

``` XML
<node pkg="pan_tilt" type="pan_tilt_node" name="pan_tilt_node" output="screen">
 <remap from="pan_tilt_node/index0_position" to="pan_tilt_node/head_position" /> </node>
```

The last tag in the file will result in the running of the rosserial node which communicates with the Arduino. You can see the argument which selects the serial port connected to the Arduino, __args="/dev/ttyUSB0"__

``` XML
<node pkg="rosserial_python" type="serial_node.py" name="serial_node" output="screen" args="/dev/ttyUSB0" />
```

The remaining folders (*include* and *src*) contain the C++ code for the package. For this package we have one C++ class, __PanTiltNode__ and a main routine contained within the pan_tilt_node.cpp file.

The main routine informs ROS of our node, creates a instance of our class which contains the code for the node, passes a callback function to the dynamic reconfiguration server and hands control to ROS spin which will handle the incoming topics and the posting of outgoing topics. 

``` C++
Add main routine
```
