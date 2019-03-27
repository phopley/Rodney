// Main control node for the Rodney robot 
#include <rodney/rodney_node.h>
#include <ros/package.h>
#include <robot_localization/SetPose.h>

// Constructor 
RodneyNode::RodneyNode(ros::NodeHandle n, ros::NodeHandle n_private)
{
    nh_ = n;
    nh_private_ = n_private;

    joystick_linear_speed_ = 0.0f;
    joystick_angular_speed_ = 0.0f;
        
    linear_mission_demand_ = 0.0f;
    angular_mission_demand_ = 0.0f;
    
    manual_locomotion_mode_ = false;
    linear_set_speed_ = 0.25f;
    angular_set_speed_ = 1.5f; 
    
    // Obtain any configuration values from the parameter server. If they don't exist use the defaults
    // Using the private node handle in this way means the <node_name> will prefix the parameter
    nh_private_.param("controller/axes/linear_speed_index", linear_speed_index_, 0);
    nh_private_.param("controller/axes/angular_speed_index", angular_speed_index_, 1);
    nh_private_.param("controller/axes/camera_x_index", camera_x_index_, 2);
    nh_private_.param("controller/axes/camera_y_index", camera_y_index_, 3);
    nh_private_.param("controller/buttons/manual_mode_select", manual_mode_select_, 0);
    nh_private_.param("controller/buttons/default_camera_pos_select", default_camera_pos_select_, 1);
    nh_private_.param("controller/buttons/lidar_enable", lidar_enable_select_, 2);
    nh_private_.param("controller/dead_zone", dead_zone_, 2000);
    nh_private_.param("teleop/max_linear_speed", max_linear_speed_, 1.0f);
    nh_private_.param("teleop/max_angular_speed", max_angular_speed_, 8.7f);
    nh_private_.param("motor/ramp/linear", ramp_for_linear_, 5.0f);
    nh_private_.param("motor/ramp/angular", ramp_for_angular_, 5.0f);
    nh_private_.param("battery/warning_level", voltage_level_warning_, 9.5f);    
    nh_private_.param("sounds/enabled", wav_play_enabled_, false);
    // TODO This would be improved by adding a service to thunderborg which allowed us to ask if the pid is enabled
    nh_.param("/thunderborg_node/pid/use_pid", pid_enabled_, false);
    
    // Obtain the filename and text for the wav files that can be played    
    nh_private_.getParam("/rodney/sounds/filenames", wav_file_names_);
    nh_private_.getParam("/rodney/sounds/text", wav_file_texts_);
     
    // Subscribe to receive keyboard input, joystick input, mission complete, battery state and remote heartbeat
    key_sub_ = nh_.subscribe("keyboard/keydown", 5, &RodneyNode::keyboardCallBack, this);
    joy_sub_ = nh_.subscribe("joy", 1, &RodneyNode::joystickCallback, this);
    mission_sub_ = nh_.subscribe("/missions/mission_complete", 5, &RodneyNode::completeCallBack, this);
    battery_status_sub_ = nh_.subscribe("main_battery_status", 1, &RodneyNode::batteryCallback, this);
    remote_heartbeat_sub_ = nh_.subscribe("remote_heartbeat", 1, &RodneyNode::remHeartbeatCallback, this);
    
    // The cmd_vel topic below is the command velocity message to the motor driver.
    // This can be created from either keyboard or game pad input when in manual mode or from the this 
    // subscribed topic when in autonomous mode. It will probably be remapped from the navigation stack
    demand_sub_ = nh_.subscribe("demand_vel", 5, &RodneyNode::motorDemandCallBack, this);

    // Advertise the topics we publish
    face_status_pub_ = nh_.advertise<std_msgs::String>("/robot_face/expected_input", 5);
    mission_pub_ = nh_.advertise<std_msgs::String>("/missions/mission_request", 10);
    cancel_pub_ = nh_.advertise<std_msgs::Empty>("/missions/mission_cancel", 5);
    ack_pub_ = nh_.advertise<std_msgs::Empty>("/missions/acknowledge", 5);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    battery_low_count_ = 0;
    mission_running_ = false;
    
    // Calculate the slope and y-intercept of the joytick input against linear speed
    lslope_ = max_linear_speed_/(MAX_AXES_VALUE_-dead_zone_);
    lyintercept_ = -(lslope_*dead_zone_);
    
    // Calculate the slope and y-intercept of the joytick input against angular speed
    aslope_ = max_angular_speed_/(MAX_AXES_VALUE_-dead_zone_);
    ayintercept_ = -(aslope_*dead_zone_);
    
    // Seed the random number generator
    srand((unsigned)time(0));
    
    last_interaction_time_ = ros::Time::now();
}
//---------------------------------------------------------------------------

void RodneyNode::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    float joystick_x_axes;
    float joystick_y_axes;
            
    // manual locomotion mode can use the joystick/game pad
    joystick_x_axes = msg->axes[angular_speed_index_];
    joystick_y_axes = msg->axes[linear_speed_index_];
    
    // Check for manual movement
    
    // Check dead zone values
    if(abs(joystick_y_axes) < dead_zone_)
    {
        joystick_linear_speed_ = 0.0f;
    }
    else
    { 
        joystick_linear_speed_ = (lslope_*(float)abs(joystick_y_axes))+lyintercept_;
        
        if(joystick_y_axes > 0.0f)
        {
            joystick_linear_speed_ = -joystick_linear_speed_;
        }
    }
       
    // Check dead zone values   
    if(abs(joystick_x_axes) < dead_zone_)
    {
        joystick_angular_speed_ = 0.0f;
    }
    else
    {
        joystick_angular_speed_ = (aslope_*(float)abs(joystick_x_axes))+ayintercept_;
        
        if(joystick_x_axes > 0.0f)
        {
            joystick_angular_speed_ = -joystick_angular_speed_;
        }
    }
    
    // Now check the joystick/game pad for manual camera movement               
    joystick_x_axes = msg->axes[camera_x_index_];
    joystick_y_axes = msg->axes[camera_y_index_];
    
    // Check dead zone values   
    if(abs(joystick_x_axes) < dead_zone_)
    {
        joystick_x_axes = 0;
    }
    
    if(abs(joystick_y_axes) < dead_zone_)
    {
        joystick_y_axes = 0;
    }  
    
    if(manual_locomotion_mode_ == true)
    {
        if((joystick_x_axes != 0) || (joystick_y_axes != 0))
        {
            std_msgs::String mission_msg;   
            mission_msg.data = "J3^";
        
            if(joystick_y_axes == 0)
            {
                mission_msg.data += "-^";
            }
            else if (joystick_y_axes > 0)
            {
                mission_msg.data += "u^";
            }
            else
            {
                mission_msg.data += "d^";        
            }
        
            if(joystick_x_axes == 0)
            {
                mission_msg.data += "-";
            }
            else if (joystick_x_axes > 0)
            {
                mission_msg.data += "r";
            }
            else
            {
                mission_msg.data += "l";        
            }
        
            mission_pub_.publish(mission_msg);
            
            last_interaction_time_ = ros::Time::now();
        }
    }
    
    // Button on controller selects manual locomotion mode
    if(msg->buttons[manual_mode_select_] == 1)
    {
        if(mission_running_ == true)
        {
            // Cancel the ongoing mission
            std_msgs::Empty empty_msg;
            cancel_pub_.publish(empty_msg);                        
        }
        
        // Reset speeds to zero           
        keyboard_linear_speed_ = 0.0f; 
        keyboard_angular_speed_ = 0.0f;
        
        manual_locomotion_mode_ = true;
        
        last_interaction_time_ = ros::Time::now(); 
    }
    
    // Button on controller selects central camera position   
    if((manual_locomotion_mode_ == true) && (msg->buttons[default_camera_pos_select_] == 1))
    {            
        std_msgs::String mission_msg;
        mission_msg.data = "J3^c^-";
        mission_pub_.publish(mission_msg);
        
        last_interaction_time_ = ros::Time::now();
    }
    
    // Button on controller selects to enable/disable the lidar function
    if((manual_locomotion_mode_ == true) && (msg->buttons[lidar_enable_select_] == 1))
    {
        std_msgs::String mission_msg;
        
        // Toggle the LIDAR on/off
        mission_msg.data = "J4";            
        
        mission_pub_.publish(mission_msg);        
        last_interaction_time_ = ros::Time::now();
    }
}
//---------------------------------------------------------------------------

void RodneyNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{
    // Check for any keys we are interested in 
    // Current keys are:
    //      'Space' - Stop the robot from moving if in manual locomotion mode
    //      'Key pad 1 and Num Lock off' - Move robot forwards and counter-clockwise if in manual locomotion mode
    //      'Key pad 2 and Num Lock off' - Move robot backwards if in manual locomotion mode
    //      'Key pad 3 and Num Lock off' - Move robot backwards and clockwise if in manual locomotion mode 
    //      'Key pad 4 and Num Lock off' - Move robot counter-clockwise if in manual locomotion mode   
    //      'Key pad 6 and Num Lock off' - Move robot clockwise if in manual locomotion mode
    //      'Key pad 7 and Num Lock off' - Move robot forwards amd counter-clockwise if in manual locomotion mode    
    //      'Key pad 8 and Num Lock off' - Move robot foward if in manual locomotion mode
    //      'Key pad 9 and Num Lock off' - Move robot forwards amd clockwise if in manual locomotion mode
    //      'Up key' - Move head/camera down in manual mode
    //      'Down key' - Move head/camera up in manual mode
    //      'Right key' - Move head/camera right in manual mode
    //      'Left key' - Move head/camera left in manual mode 
    //      'Key pad +' - Increase linear speed by 10% (speed when using keyboard for teleop)
    //      'Key pad -' - Decrease linear speed by 10% (speed when using keyboard for teleop)
    //      'Key pad *' - Increase angular speed by 10% (speed when using keyboard for teleop)
    //      'Key pad /' - Decrease angular speed by 10% (speed when using keyboard for teleop)   
    //      '1' to '9' - Run a mission (1 -9)
    //      'a' or 'A' - Some missions require the user to send an acknowledge
    //      'c' or 'C' - Cancel current mission
    //      'd' or 'D' - Move head/camera to the default position in manual mode
    //      'l' or 'L' - Toggle LIDAR on/off 
    //      'm' or 'M' - Set locomotion mode to manual
    //      'r' or 'R' - Reset the odometry
    
    // Check for a number key (not key pad) with modifiers apart from num lock is allowed
    if(((msg->code >= keyboard::Key::KEY_1) && (msg->code <= keyboard::Key::KEY_9)) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {    
        // Start a mission 
        std_msgs::String mission_msg;
        mission_msg.data = "M" + std::to_string(msg->code-keyboard::Key::KEY_0);
        mission_pub_.publish(mission_msg);
                    
        mission_running_ = true; 
        manual_locomotion_mode_ = false;        
        
        last_interaction_time_ = ros::Time::now();                       
    }
    else if((msg->code == keyboard::Key::KEY_c) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {          
        // 'c' or 'C', cancel mission if one is running
        if(mission_running_ == true)
        {
            std_msgs::Empty empty_msg;
            cancel_pub_.publish(empty_msg);
        }
        
        last_interaction_time_ = ros::Time::now();        
    }
    else if((msg->code == keyboard::Key::KEY_a) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {
        // 'a' or 'A', acknowledge a mission step
        if(mission_running_ == true)
        {
            std_msgs::Empty empty_msg;
            ack_pub_.publish(empty_msg);                
        }
        
        last_interaction_time_ = ros::Time::now();        
    }
    else if((msg->code == keyboard::Key::KEY_d) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {          
        // 'd' or 'D', Move camera to default position
        if(manual_locomotion_mode_ == true)
        {            
            std_msgs::String mission_msg;
            mission_msg.data = "J3^c^-";
            mission_pub_.publish(mission_msg);
        }    
        
        last_interaction_time_ = ros::Time::now();   
    } 
    else if((msg->code == keyboard::Key::KEY_l) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {
        if(manual_locomotion_mode_ == true)
        {
            std_msgs::String mission_msg;
        
            // Toggle the LIDAR on/off
            mission_msg.data = "J4";            
        
            mission_pub_.publish(mission_msg);        
            last_interaction_time_ = ros::Time::now();
        }
    }      
    else if((msg->code == keyboard::Key::KEY_m) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {
        // 'm' or 'M', set locomotion mode to manual (any missions going to auto should set manual_locomotion_mode_ to false)
        // When in manual mode user can teleop Rodney with keyboard or joystick
        if(mission_running_ == true)
        {
            // Cancel the ongoing mission
            std_msgs::Empty empty_msg;
            cancel_pub_.publish(empty_msg);                        
        }
        
        // Reset speeds to zero           
        keyboard_linear_speed_ = 0.0f; 
        keyboard_angular_speed_ = 0.0f;
        
        manual_locomotion_mode_ = true;
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_r) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {
        // 'r' or 'R', reset odometry
        ros::ServiceClient client = nh_.serviceClient<robot_localization::SetPose>("set_pose");
        robot_localization::SetPose srv;
        srv.request.pose.header.frame_id ="odom";
        client.call(srv);
    }
    else if((msg->code == keyboard::Key::KEY_KP1) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 1 on keypad without num lock
        // If in manual locomotion mode this is an indication to move backwards and counter-clockwise by the current set speeds
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = -linear_set_speed_;                        
            keyboard_angular_speed_ = -angular_set_speed_;        
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_KP2) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 2 on keypad without num lock
        // If in manual locomotion mode this is an indication to move backwards by the current linear set speed
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = -linear_set_speed_;        
            keyboard_angular_speed_ = 0.0f;            
        }
        
        last_interaction_time_ = ros::Time::now();
    }  
    else if((msg->code == keyboard::Key::KEY_KP3) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 3 on keypad without num lock
        // If in manual locomotion mode this is an indication to move backwards and clockwise by the current set speeds
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = -linear_set_speed_;
            keyboard_angular_speed_ = angular_set_speed_;                    
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_KP4) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 4 on keypad without num lock
        // If in manual locomotion mode this is an indication to turn counter-clockwise (spin on spot) by the current angular set speed
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = 0.0f;
            keyboard_angular_speed_ = angular_set_speed_;                    
        }
        
        last_interaction_time_ = ros::Time::now();
    } 
    else if((msg->code == keyboard::Key::KEY_KP6) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 6 on keypad without num lock
        // If in manual locomotion mode this is an indication to turn clockwise (spin on spot) by the current angular set speed
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = 0.0f;  
            keyboard_angular_speed_ = -angular_set_speed_;                  
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_KP7) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 7 on keypad without num lock
        // If in manual locomotion mode this is an indication to move forwards and counter-clockwise by the current set speeds
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = linear_set_speed_; 
            keyboard_angular_speed_ = angular_set_speed_;                   
        }
        
        last_interaction_time_ = ros::Time::now();
    }    
    else if((msg->code == keyboard::Key::KEY_KP8) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 8 on keypad without num lock
        // If in manual locomotion mode this is an indication to move forward by the current linear set speed
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = linear_set_speed_; 
            keyboard_angular_speed_ = 0.0f;                   
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if((msg->code == keyboard::Key::KEY_KP9) && ((msg->modifiers & keyboard::Key::MODIFIER_NUM) == 0))
    {
        // Key 9 on keypad without num lock
        // If in manual locomotion mode this is an indication to move forwards and clockwise by the current set speeds
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_ = linear_set_speed_; 
            keyboard_angular_speed_ = -angular_set_speed_;                   
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if(msg->code == keyboard::Key::KEY_SPACE)
    {
        // Space key
        // If in manual locomotion stop the robot movment 
        if(manual_locomotion_mode_ == true)
        {
            keyboard_linear_speed_= 0.0f;     
            keyboard_angular_speed_ = 0.0f;               
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if(msg->code == keyboard::Key::KEY_KP_PLUS)
    {
        // '+' key on num pad
        // If in manual locomotion increase linear speed by 10%
        if(manual_locomotion_mode_ == true)
        {
            linear_set_speed_ += ((10.0/100.0) * linear_set_speed_);
            ROS_INFO("Linear Speed now %f", linear_set_speed_);
        }  
        
        last_interaction_time_ = ros::Time::now();  
    }
    else if(msg->code == keyboard::Key::KEY_KP_MINUS)
    {
        // '-' key on num pad
        // If in manual locomotion decrease linear speed by 10%
        if(manual_locomotion_mode_ == true)
        {
            linear_set_speed_ -= ((10.0/100.0) * linear_set_speed_);
            ROS_INFO("Linear Speed now %f", linear_set_speed_);
        }  
        
        last_interaction_time_ = ros::Time::now();      
    }
    else if(msg->code == keyboard::Key::KEY_KP_MULTIPLY)
    {
        // '*' key on num pad
        // If in manual locomotion increase angular speed by 10%
        if(manual_locomotion_mode_ == true)
        {
            angular_set_speed_ += ((10.0/100.0) * angular_set_speed_);
            ROS_INFO("Angular Speed now %f", angular_set_speed_);
        }    
        
        last_interaction_time_ = ros::Time::now();
    }
    else if(msg->code == keyboard::Key::KEY_KP_DIVIDE)
    {
        // '/' key on num pad        
        // If in manual locomotion decrease angular speed by 10%
        if(manual_locomotion_mode_ == true)
        {
            angular_set_speed_ -= ((10.0/100.0) * angular_set_speed_);
            ROS_INFO("Angular Speed now %f", angular_set_speed_);
        }   
        
        last_interaction_time_ = ros::Time::now(); 
    }    
    else if(msg->code == keyboard::Key::KEY_UP)
    {
        // Up Key
        // This is a simple job not a mission - move the head/camera down
        if(manual_locomotion_mode_ == true)
        {            
            std_msgs::String mission_msg;
            mission_msg.data = "J3^d^-";
            mission_pub_.publish(mission_msg);
        }
        
        last_interaction_time_ = ros::Time::now();
    }
    else if(msg->code == keyboard::Key::KEY_DOWN)
    {
        // Down Key
        // This is a simple job not a mission - move the head/camera up
        if(manual_locomotion_mode_ == true)
        {
            std_msgs::String mission_msg;
            mission_msg.data = "J3^u^-";
            mission_pub_.publish(mission_msg);
        }
        
        last_interaction_time_ = ros::Time::now();
    }  
    else if(msg->code == keyboard::Key::KEY_LEFT)
    {
        // Left key
        // This is a simple job not a mission - move the head/camera left
        if(manual_locomotion_mode_ == true)
        {
            std_msgs::String mission_msg;
            mission_msg.data = "J3^-^l";
            mission_pub_.publish(mission_msg);
        }
        
        last_interaction_time_ = ros::Time::now();
    }       
    else if(msg->code == keyboard::Key::KEY_RIGHT)
    {
        // Right Key
        // This is a simple job not a mission - move the head/camera right
        if(manual_locomotion_mode_ == true)
        {
            std_msgs::String mission_msg;
            mission_msg.data = "J3^-^r";
            mission_pub_.publish(mission_msg);
        }
        
        last_interaction_time_ = ros::Time::now();
    }                             
    else
    {
        ;
    } 
}
//---------------------------------------------------------------------------

// Callback for when motor demands received in autonomous mode
void RodneyNode::motorDemandCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{ 
    linear_mission_demand_ = msg->linear.x;
    angular_mission_demand_ = msg->angular.z;
}
//---------------------------------------------------------------------------

// Callback for remote heartbeat
void RodneyNode::remHeartbeatCallback(const std_msgs::Empty::ConstPtr& msg)
{
    // Remote heartbeat received store the time
    remote_heartbeat_time_ = ros::Time::now();
}

// Callback for main battery status
void RodneyNode::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{ 
    // Convert float to string with two decimal places
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << msg->voltage;
    std::string voltage = ss.str();
    
    std_msgs::String status_msg;
    
    // Publish battery voltage to the robot face
    // However the '.' will be used by the face to change the expression to neutral so we will replace with ','
    replace(voltage.begin(), voltage.end(), '.', ',');
    
    if(msg->voltage > voltage_level_warning_)
    {
        status_msg.data = "Battery level OK ";
        battery_low_count_ = 0;
    }
    else
    {
        // If the battery level goes low we wait a number of messages to confirm it was not a dip as the motors started
        if(battery_low_count_ > 1)
        {
        
            status_msg.data = "Battery level LOW ";
        
            // Speak warning every 5 minutes        
            if((ros::Time::now() - last_battery_warn_).toSec() > (5.0*60.0))
            {
                last_battery_warn_ = ros::Time::now();
            
                std_msgs::String mission_msg;
                mission_msg.data = "J2^battery level low^Battery level low:(";
                mission_pub_.publish(mission_msg);
            }
        }
        else
        {
            battery_low_count_++;
        }
    }
    
    status_msg.data += voltage + "V";                                 
    face_status_pub_.publish(status_msg);
}
//---------------------------------------------------------------------------

void RodneyNode::completeCallBack(const std_msgs::String::ConstPtr& msg)
{
    mission_running_ = false;
    
    last_interaction_time_ = ros::Time::now();
}
//---------------------------------------------------------------------------

void RodneyNode::checkTimers(void)
{
    /* Check time since last interaction */
    if((wav_play_enabled_ == true) && (mission_running_ == false) && ((ros::Time::now() - last_interaction_time_).toSec() > (15.0*60.0)))
    {
        last_interaction_time_ = ros::Time::now();
        
        // Use a random number to pick the wav file
        int random = (rand()%wav_file_names_.size())+1;                
         
        // This is a simple job not a mission
        std_msgs::String mission_msg;
        std::string path = ros::package::getPath("rodney");
        mission_msg.data = "J1^" + path + "/sounds/" + wav_file_names_[std::to_string(random)] + 
                           "^" + wav_file_texts_[std::to_string(random)];        
        mission_pub_.publish(mission_msg);         
    }
}
//---------------------------------------------------------------------------

void RodneyNode::sendTwist(void)
{
    geometry_msgs::Twist target_twist;
    
    // If in manual locomotion mode use keyboard or joystick data
    if(manual_locomotion_mode_ == true)
    {
        // Only allow stored keyboard or joystick values to set  
        // the velocities if the remote heartbeat is running
        if((ros::Time::now() - remote_heartbeat_time_).toSec() < 1.0)
        {        
            // Publish message based on keyboard or joystick speeds
            if((keyboard_linear_speed_ == 0) && (keyboard_angular_speed_ == 0))
            {
                // Use joystick values
                target_twist.linear.x = joystick_linear_speed_;
                target_twist.angular.z = joystick_angular_speed_;            
            }
            else
            {
                // use keyboard values
                target_twist.linear.x = keyboard_linear_speed_;
                target_twist.angular.z = keyboard_angular_speed_;                   
            }
        }
        else
        {
            // Lost connection with remote workstation so zero the velocities
            target_twist.linear.x = 0.0;
            target_twist.angular.z = 0.0; 
        }
    }
    else
    {
        // Use mission demands (autonomous)
        target_twist.linear.x = linear_mission_demand_;
        target_twist.angular.z = angular_mission_demand_;
    }

    // If not using the PID ramp to the target value. 
    if (false == pid_enabled_)
    {
        ros::Time time_now = ros::Time::now();
        
        // Ramp towards are required twist velocities
        last_twist_ = rampedTwist(last_twist_, target_twist, last_twist_send_time_, time_now);
        
        last_twist_send_time_ = time_now;
        
        // Publish the Twist message using the ramp value
        twist_pub_.publish(last_twist_);
    }
    else
    {
        // Publish the Twist message using the target value
        twist_pub_.publish(target_twist);
    }        
}
//---------------------------------------------------------------------------

geometry_msgs::Twist RodneyNode::rampedTwist(geometry_msgs::Twist prev, geometry_msgs::Twist target,
                                             ros::Time time_prev, ros::Time time_now)
{
    // Ramp the angular and linear values towards the tartget values
    geometry_msgs::Twist retVal;
    
    retVal.angular.z = rampedVel(prev.angular.z, target.angular.z, time_prev, time_now, ramp_for_angular_);
    retVal.linear.x = rampedVel(prev.linear.x, target.linear.x, time_prev, time_now, ramp_for_linear_);
    
    return retVal;
}
//---------------------------------------------------------------------------

float RodneyNode::rampedVel(float velocity_prev, float velocity_target, ros::Time time_prev, ros::Time time_now,
                            float ramp_rate)
{
    // Either move towards the velocity target or if difference is small jump to it
    float retVal;    
    float sign;
    float step = ramp_rate * (time_now - time_prev).toSec();
    
    if(velocity_target > velocity_prev)
    {
        sign = 1.0f;
    }
    else
    {
        sign = -1.0f;
    }
    
    float error = std::abs(velocity_target - velocity_prev);
    
    if(error < step)
    {
        // Can get to target in this within this time step
        retVal = velocity_target;
    }
    else
    {
        // Move towards our target
        retVal = velocity_prev + (sign * step);
    }        
    
    return retVal;
}
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "rodney");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");    
    RodneyNode rodney_node(nh, nh_private);   
    std::string node_name = ros::this_node::getName();
    ROS_INFO("%s started", node_name.c_str());
	
    ros::Rate r(20); // 20Hz	    
    
    while(ros::ok())
    {
        rodney_node.sendTwist();
        rodney_node.checkTimers();
        
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;    
}

