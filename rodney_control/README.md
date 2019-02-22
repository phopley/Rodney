# rodney_control
This Arduino/Teensy sketch, rodney_control, runs using rosserial and is a ROS node which:
* Controls upto four RC Servos on the servo topic
* Publishes the tacho on the tacho topic monitoring two motors with Hall sensors

The node subscribes to the servo topic and acts on a rodney_msgs::servo_array message.
This message contains two elements, index and angle. Index references the servos 0-3 and
angle is the angle to set the servo to 0-180.

The connections to a Teensy 3.5 are:
* Pin 0 (INT)   -> used for monitoring right motor speed
* Pin 1 (Input)   -> used for sensing right motor direction 
* Pin 3 (INT)   -> used for monitoring left motor speed
* Pin 4 (Input) -> used for sensing left motor direction 
* Pin 20 (PWM)  -> servo indexed 3
* Pin 21 (PWM)  -> servo indexed 2 
* Pin 22 (PWM)  -> servo indexed 1
* PIN 23 (PWM)  -> servo indexed 0

