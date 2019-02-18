# rodney

ROS node responsible for the main control of Rodney robot

## Running the Node

Once you have the node built you can run the rodney robot by launching the rodney.launch file.

## Node Information
Topics:

* `keyboard/keydown`:  
  Subscribes `keyboard/Key` keyboard key presses
  
* `joy`:  
  Subscribes `sensor_msgs/Joy` Joystick/game pad controller input
  
* `/missions/mission_complete`:  
  Subscribes `std_msgs/String` Message indicating that the current mission completed
  
* `main_battery_status`:  
  Subscribes `sensor_msgs/BatteryState` Status of the main battery
  
* `demand_vel`:  
  Subscribes `geometry_msgs/Twist` Velocity demands from autonomous sub-system
  
* `/robot_face/expected_input`:  
  Publishes `std_msgs/String` Status messages
  
* `missions/mission_request`:  
  Publishes `std_msgs/String` Request to carry out robot missions or simple jobs
  
* `/missions/mission_cancel`:  
  Publishes `std_msgs/Empty` Request to cancel the current robot mission
  
* `/missions/acknowledge`:  
  Publishes `std_msgs/Empty` Acknowledge that a mission can move on
  
* `cmd_vel`:  
  Publishes `geometry_msgs/Twist` Velocity demands from either manaul input or the autonomous sub-system

* `/commands/reset_odometry`:  
  Publishes `std_msgs/Empty` Request to reset the odmoetry values
  
Parameters:

* `/controller/axes/linear_speed_index`: Index of the controller axes used for requesting forward and backward movement. 
* `/controller/axes/angular_speed_index`: Index of the controller axes used for requesting clockwise and anti-clockwise movement.
* `/controller/axes/camera_x_index`: Index of the controller axes used for requesting head up and down movement. 
* `/controller/axes/camera_y_index`: Index of the controller axes used for requesting head left and right movement.
* `/controller/buttons/manual_mode_select`: Index of the controller button used for setting manaul mode.
* `/controller/buttons/default_camera_pos_select`: Index of the controller button used for rquesting that the head returns to default position.
* `/controller/dead_zone`: Controller axes dead zone value, until the controller value is greater than this value it is considered to be at zero.
* `/teleop/max_linear_speed`: The linear velocity which is requested when the controller axes is at its maxium range.
* `/teleop/max_angular_speed`: The angular velocity which is requested when the controller axes is at its maxium range.
* `/motor/ramp/linear`: The ramp rate used to increase or decrease the linear velocity.
* `/motor/ramp/angular`: The ramp rate used to increase or decrease the angular velocity.
* `/battery/warning_level`: The battery voltage level at which to issue a low battery warning. 
* `/sounds/enabled`: If true the robot is free to use stored wav file to get attention when inactive.
* `/sounds/filenames`: A list of wav filenames to choose from when inactive
* `/sounds/text`: A list of speeches which match the contents of the wav file names.

## Arduino Sketch
The `rodney_control` directory contains an Arduino sketch `rodney_control.ino` which handles some of the rodney robot function e.g. servo movements, motor encoder readings and LIDAR data.
