^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rodney
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2019-01-24)
------------------
* Improved conversion from joystick position to speed command
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
