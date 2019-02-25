^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rodney
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unreleased
------------------
* Added LIDAR to URDF model
* "Arduino" code changed to use either an Arduino Nano or a Teensy. Nano uses smaller buffer sizes a Teensy use higher baud rate
* Increased tacho message publish rate to 40Hz on a Teensy
* In the urdf file added origin to each inertial tag where it is not left as default
* Added functionality to manually enable to LIDAR from keyboard or joystick
* Changed head points in URDF model from EfforJointInterface to PositionJointInterface
* Moved scripts to a scripts folder and create rules for remapping serial ports

0.2.0 (2019-01-24)
------------------
* Improved conversion from joystick position to speed command
* Ramp to target velocity now only used if PID disabled
* Added encoder code to the Arduino sketch
* Added the Thunderborg node to the launch file
* Added Thunderborg and tacho_msgs to clonepackages and createlinks file
* Added camera to urdf model

0.1.3 (2019-01-13)
------------------
* Removed republish from the launch file
* Angular set speed default now 2.5 radians/sec instead of 1.0
* Added publishing the reset_odometry command if keyboard 'R' or 'r' pressed
* Added source list parameter to rviz.launch
* Tilt default at power up in rodney_control.ino now set to 120
* joystick_linear_speed_ and joystick_angular_speed_ set to 0.0 in constructor to fix bug if joystick node not running
* Removed mission 3

0.1.2 (2018-12-05)
------------------
* Set mission_running_ to false in contructor
* Added code to start mission 3 and acknowledge missions steps

0.1.1 (2018-11-12)
------------------
* Moved Arduino sketch into package
* Updates to documentation
* Moved the scaning for faces action client to rondey_missions package
* Added use of speech and facial expression
* Added rviz files for robot modelling
* Added teleop with keyboard and game controller
* Added battery status reporting
* Added wav file playback if inactive

0.1.0 (2018-06-14)
------------------
* First formal release of the package
