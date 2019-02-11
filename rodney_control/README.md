# rodney_control
This Arduino/Teensy sketch, rodney_control, runs using rosserial and is a ROS node which:
* Controls upto four RC Servos on the servo topic
* Publishes the tacho on the tacho topic monitoring two motors with Hall sensors
* Controls a servo attached to a LIDAR and takes range measurments from the LIDAR publishing them on the laserScan topic

The node subscribes to the servo topic and acts on a rodney_msgs::servo_array message.
This message contains two elements, index and angle. Index references the servos 0-3 and
angle is the angle to set the servo to 0-180.

The node also subscribes to the /commands/lidar_enable topic the acts on a std_msgs::Bool message.
If the data in the message is true the use of the LIDAR is enabled, otherwise it is disabled.

The connections to a Teensy 3.5 are:
* Pin 0 (INT)   -> used for monitoring right motor speed
* Pin 1 (Input)   -> used for sensing right motor direction
* Pin 2 (PWM)   -> LIDAR Servo control signal 
* Pin 3 (INT)   -> used for monitoring left motor speed
* Pin 4 (Input) -> used for sensing left motor direction 
* Pin 18 (SCL)  -> LIDAR SCL (Green wire on Garmin LIDAR-Lite V3)
* Pin 19 (SDA)  -> LIDAR SDA (Blue wire on Garmin LIDAR-Lite V3)
* Pin 20 (PWM)  -> servo indexed 3
* Pin 21 (PWM)  -> servo indexed 2 
* Pin 22 (PWM)  -> servo indexed 1
* PIN 23 (PWM)  -> servo indexed 0

