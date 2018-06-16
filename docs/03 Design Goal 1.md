# Design Goals
The robot project will be developed using a number of design goals.
## Design Goal 1
Use Case - On power up look around using the camera, search for faces, attempt to identify any seen and display a message for any identified. This will require:
- Control of head/camera using RC servos
- Access to Pi Camera
- Facial detections and recognition
### Pan & Tilt
To control the head/camera we need a pan and tilt device which will require two RC servos. I expect that the project will also in the future require a second device for a LIDAR (Light Detection and Ranging) sensor. We therefore straight away require four PWM outputs, not to mention any required for motors in the future. The Raspberry Pi only has one hardware PWM, although software PWMs could be used, so I'm going to pass the control of the servos off to a second board.

You could use a purpose built board like one available from PiBorg, the [UltraBorg](https://www.piborg.org/sensors-1136/ultraborg "UltraBorg"). Using this board you can connect up to four servos and four HC-SR04 ultrasonic devices to the Raspberry Pi using an I2C bus.

However, since I have a number of Arduino Nano's available from a previous project I'm going to make use of one of those. To attach and control the Arduino I'm also going to make use of a ROS package designed to communicate with an Arduino its USB serial port. This is our first example of being able to take advantage of a ROS package written by some else and made available from the ROS Wiki website. [rosserial_arduino](http://wiki.ros.org/rosserial_arduino "rosserial_arduino")
