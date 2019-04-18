^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rodney
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2019-04-18)
------------------
* Added configuration and launching of the nav stack

1.0.0 (2019-04-10)
------------------
* Added IMU calibration node to launch file
* Added remote heartbeat message which will be run on remote workstation when teleoping
* Added RPLidar and scan filter nodes to rodney launch file
* Added functionality to manually enable the LIDAR from keyboard or joystick
* Added LIDAR to URDF model
* Added rodney.xacro and now auto generate rodney.urdf file
* "Arduino" code changed to use a Teensy and now reads IMU data
* Changed head joints in URDF model from EfforJointInterface to PositionJointInterface
* Moved scripts to a scripts folder and create rules for remapping serial ports
* Now using use_ramp parameter instead of use_pid


0.3.2 (2019-03-19)
------------------
* Reset of odom now uses a service call to robot_localization not /commands/reset_raw_odometry

0.3.1 (2019-03-18)
------------------
* Topic /commands/reset_odometry should have been renamed /commands/reset_raw_odometry

0.3.0 (2019-03-17)
------------------
* Added ekf_localization_node from the robot_localization package to the launch file
* Added the robot_localization.yaml file to the config folder

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
