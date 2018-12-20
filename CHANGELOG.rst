^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rodney
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unreleased
------------------
* Update to rodney_control.ino to include rpm message
* Tilt default at power up in rodney_control.ino now set to 90
* Added tacho_msgs and thunderborg to clonepackages and createlinks
* joystick_linear_speed_ and joystick_angular_speed_ set to 0.0 in constructor to fix bug if joystick node not running

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
