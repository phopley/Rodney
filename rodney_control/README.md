# rodney_control
This Arduino sketch, rodney_control, runs using rosserial and is a ROS node which controls upto four RC Servo's and reads two Hall sensors for motor encoders.

The node subscribes to the servo topic which consists of a servo_msgs::servo_array message. This message contains two elements, index and angle. The index addresses the servo 0-3 and the angle gives the servo required angle 0-180.

It publishes the tacho message which consists of a tacho_msgs::tacho message. This message contains two elements that give the left and right motor rpm.
