// Main control node for the Rodney robot 
#include <rodney/rodney_node.h>
#include <std_msgs/Empty.h>
#include <ros/package.h>

// Constructor 
RodneyNode::RodneyNode(ros::NodeHandle n)
{
    nh_ = n;
    
    linear_mission_demand_ = 0.0f;
    angular_mission_demand_ = 0.0f;
    
    manual_locomotion_mode_ = false;
    linear_set_speed_ = 0.5f;
    angular_set_speed_ = 1.0f;
    
    linear_speed_index_ = 0;
    angular_speed_index_ = 1;
    manual_mode_select_ = 0;
    
    camera_x_index_ = 2;
    camera_y_index_ = 3;
    
    max_linear_speed_ = 3;
    max_angular_speed_ = 3;
    
    dead_zone_ = 2000;
    
    ramp_for_linear_ = 5.0f;
    ramp_for_angular_ = 5.0f;   
    
    // Obtain any configuration values from the parameter server. If they don't exist use the defaults above
    nh_.param("/controller/axes/linear_speed_index", linear_speed_index_, linear_speed_index_);
    nh_.param("/controller/axes/angular_speed_index", angular_speed_index_, angular_speed_index_);
    nh_.param("/controller/buttons/manual_mode_select", manual_mode_select_, manual_mode_select_);
    nh_.param("/controller/dead_zone", dead_zone_, dead_zone_);
    nh_.param("/teleop/max_linear_speed", max_linear_speed_, max_linear_speed_);
    nh_.param("/teleop/max_angular_speed", max_angular_speed_, max_angular_speed_);
    nh_.param("/motor/ramp/linear", ramp_for_linear_, ramp_for_linear_);
    nh_.param("/motor/ramp/angular", ramp_for_angular_, ramp_for_angular_);
    nh_.param("/motor/ramp/camera_x_index_", camera_x_index_, camera_x_index_);
    nh_.param("/motor/ramp/camera_y_index_", camera_y_index_, camera_y_index_);    
     
    // Subscribe to receive keyboard input, joystick input and mission complete
    key_sub_ = nh_.subscribe("keyboard/keydown", 5, &RodneyNode::keyboardCallBack, this);
    joy_sub_ = nh_.subscribe("joy", 1, &RodneyNode::joystickCallback, this);
    mission_sub_ = nh_.subscribe("/missions/mission_complete", 5, &RodneyNode::completeCallBack, this);

    // Advertise the topics we publish
    face_status_pub_ = nh_.advertise<std_msgs::String>("/robot_face/expected_input", 5);
    mission_pub_ = nh_.advertise<std_msgs::String>("/missions/mission_request", 10);
    cancel_pub_ = nh_.advertise<std_msgs::Empty>("/missions/mission_cancel", 5);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}
//---------------------------------------------------------------------------

void RodneyNode::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    float joystick_x_axes;
    float joystick_y_axes;
            
    // manual locomotion mode can use the joystick/game pad
    joystick_x_axes = msg->axes[angular_speed_index_];
    joystick_y_axes = msg->axes[linear_speed_index_];
        
    // Check dead zones    
    if(abs(joystick_x_axes) < dead_zone_)
    {
        joystick_x_axes = 0;
    }
    
    if(abs(joystick_y_axes) < dead_zone_)
    {
        joystick_y_axes = 0;
    }    
    
    if(joystick_y_axes != 0)
    {      
        joystick_linear_speed_ = -(joystick_y_axes*(max_linear_speed_/(float)MAX_AXES_VALUE_));
    }
    else
    {
        joystick_linear_speed_ = 0;
    }
    
    if(joystick_x_axes != 0)
    {
        joystick_angular_speed_ = -(joystick_x_axes*(max_angular_speed_/(float)MAX_AXES_VALUE_));
    }
    else
    {
        joystick_angular_speed_ = 0;
    }
    
    // Now check the joystick/game pad for manual camera movement               
    joystick_x_axes = msg->axes[camera_x_index_];
    joystick_y_axes = msg->axes[camera_y_index_];
    if((joystick_x_axes != 0) && (joystick_y_axes != 0) && (manual_locomotion_mode_ == true))
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
    }
    
    // Button 'A' on X-Box 360 controller selects manual locomotion mode
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
    //      'Up key' - Move head/camera up in manual mode
    //      'Down key' - Move head/camera down in manual mode
    //      'Right key' - Move head/camera right in manual mode
    //      'Left key' - Move head/camera left in manual mode 
    //      'Key pad +' - Increase linear speed by 10% (speed when using keyboard for teleop)
    //      'Key pad -' - Decrease linear speed by 10% (speed when using keyboard for teleop)
    //      'Key pad *' - Increase angular speed by 10% (speed when using keyboard for teleop)
    //      'Key pad /' - Decrease angular speed by 10% (speed when using keyboard for teleop)   
    //      '2' - Run mission 2    
    //      'c' or 'C' - Cancel current mission
    //      'm' or 'M' - Set locomotion mode to manual
    //      's' or 'S' - Currently used to test the status indication
    //      't' or 'T' - Currently used to test the speech functionality
    //      'w' or 'W' - Currently used to test wav file play back

    // Check for key 2 with no modifiers apart from num lock is allowed
    if((msg->code == keyboard::Key::KEY_2) && ((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0))
    {
        // '2', start a complete scan looking for faces (mission 2)
        std_msgs::String mission_msg;
        mission_msg.data = "M2";
        mission_pub_.publish(mission_msg);
                    
        mission_running_ = true;        
    }
    else if((msg->code == keyboard::Key::KEY_c) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {          
        // 'c' or 'C', cancel mission if one is running
        if(mission_running_ == true)
        {
            std_msgs::Empty empty_msg;
            cancel_pub_.publish(empty_msg);
        }        
    }
    else if((msg->code == keyboard::Key::KEY_s) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {
        // 's' or 'S', temporary code to test status            
        // Publish staright to the robot face    
        std_msgs::String status_msg;            
        status_msg.data = "Rodney";
        face_status_pub_.publish(status_msg);
    }    
    else if((msg->code == keyboard::Key::KEY_w) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {
        // 'w' or 'W', temporary code to test wav playing
        // This is a simple job not a mission
        std_msgs::String mission_msg;
        std::string path = ros::package::getPath("rodney");
        mission_msg.data = "J1^" + path + "/sounds/lost_in_space_danger.wav" + "^Danger Will Robinson danger:)";
        ROS_INFO("%s", path.c_str());
        mission_pub_.publish(mission_msg);            
    }
    else if((msg->code == keyboard::Key::KEY_t) && ((msg->modifiers & ~RodneyNode::SHIFT_CAPS_NUM_LOCK_) == 0))
    {
        // 't' or 'T', temporary code to test speech
        // This is a simple job not a mission
        std_msgs::String mission_msg;
        mission_msg.data = "J2^happy birthday u an^Happy birthday Iwan:)";
        mission_pub_.publish(mission_msg);
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
    }    
    else if(msg->code == keyboard::Key::KEY_UP)
    {
        // Up Key
        // This is a simple job not a mission - move the head/camera up
        if(manual_locomotion_mode_ == true)
        {            
            std_msgs::String mission_msg;
            mission_msg.data = "J3^u^-";
            mission_pub_.publish(mission_msg);
        }
    }
    else if(msg->code == keyboard::Key::KEY_DOWN)
    {
        // Down Key
        // This is a simple job not a mission - move the head/camera down
        if(manual_locomotion_mode_ == true)
        {
            std_msgs::String mission_msg;
            mission_msg.data = "J3^d^-";
            mission_pub_.publish(mission_msg);
        }
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
    }                             
    else
    {
        ;
    } 
}
//---------------------------------------------------------------------------

void RodneyNode::completeCallBack(const std_msgs::String::ConstPtr& msg)
{
    mission_running_ = false;
}
//---------------------------------------------------------------------------

void RodneyNode::sendTwist(void)
{
    geometry_msgs::Twist target_twist;
    
    // If in manual locomotion mode use keyboard or joystick data
    if(manual_locomotion_mode_ == true)
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
        // Use mission demands (autonomous)
        target_twist.linear.x = linear_mission_demand_;
        target_twist.angular.z = angular_mission_demand_;
    }
    
    ros::Time time_now = ros::Time::now();
        
    // Ramp towards are required twist velocities
    last_twist_ = rampedTwist(last_twist_, target_twist, last_twist_send_time_, time_now);
        
    last_twist_send_time_ = time_now;
        
    // Publish the Twist message
    twist_pub_.publish(last_twist_);
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
    ros::NodeHandle n;    
    RodneyNode rodney_node(n);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
	
	ros::Rate r(20); // 20Hz	    
    
    while(ros::ok())
    {
        rodney_node.sendTwist();
        
        ros::spinOnce();
        r.sleep();
    }
    
return 0;    
}

