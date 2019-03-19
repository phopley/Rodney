#ifndef RODNEY_NODE_H_
#define RODNEY_NODE_H_

#include <ros/ros.h>
#include <keyboard/Key.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

class RodneyNode
{
public:
    RodneyNode(ros::NodeHandle n);
    void sendTwist(void);
    void checkTimers(void);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher face_status_pub_;    // Topic to publish status to be displayed by RobotFace
    ros::Publisher mission_pub_;        // Topic to start a mission or small task
    ros::Publisher cancel_pub_;         // Topic to cancel a mission
    ros::Publisher ack_pub_;            // Topic for user to acknowledge some mission sections
    ros::Publisher twist_pub_;          // Topic to publish twist message
    ros::Subscriber key_sub_;           // Topic for keyboard input
    ros::Subscriber joy_sub_;           // Topic for joystick/game pad input
    ros::Subscriber mission_sub_;       // Topic for mission complete indication
    ros::Subscriber demand_sub_;        // Topic for autonomous motor demands
    ros::Subscriber battery_status_sub_;// Topic to monitor battery status for the main battery
    ros::Subscriber remote_heartbeat_sub_;  // Topic to monitor the remote heartbeat when teleoping
    ros::Time last_twist_send_time_;    // Time of previous message
    ros::Time last_battery_warn_;       // Time of the last spoken battery warning message
    ros::Time last_interaction_time_;   // Time that a human last interacted with the robot
    ros::Time remote_heartbeat_time_;   // Time that the last remote heartbeat message was received
    
    geometry_msgs::Twist last_twist_;   // The last Twist message sent   
    
    std::map<std::string,std::string> wav_file_names_; 
    std::map<std::string,std::string> wav_file_texts_;  
            
    bool mission_running_;
    bool manual_locomotion_mode_;
    bool wav_play_enabled_;
    bool pid_enabled_;
    
    unsigned int battery_low_count_; // Counter for low battery low messages   
 
    int linear_speed_index_;   // Controller axes index for linear speed
    int angular_speed_index_;  // Controller axes index for angular speed
    int manual_mode_select_;   // Controller button index for manual mode (teleop)
    int camera_x_index_;       // Controller axes index for x direction head/camera
    int camera_y_index_;       // Controller axes index for y direction head/camera
    int default_camera_pos_select_; // Controller button index for moving camera to default position
    int lidar_enable_select_;       // Controller button index for enable/disable LIDAR function
    
    int dead_zone_;                     // Controller dead zone value    
    
    float linear_set_speed_;            // Current linear set speed for keyboard manual mode locomoton
    float angular_set_speed_;           // Current angular set speed for keboard manual mode locomoton   
    float keyboard_linear_speed_;       // Linear speed from keyboard
    float keyboard_angular_speed_;      // Anular speed from keyboard
    float joystick_linear_speed_;       // Linear speed from joystick/game pad
    float joystick_angular_speed_;      // Anular speed from joystick/game pad
    float max_linear_speed_;            // Max linear speed in manual mode (teleop)
    float max_angular_speed_;           // Max angular speed in manual mode (teleop)
    float linear_mission_demand_;       // Linear speed demand when autonomous
    float angular_mission_demand_;      // Angular speed demand when autonomous
    float ramp_for_angular_;            // Ramp rate for angular velocities
    float ramp_for_linear_;             // Ramp rate for linear velocities
    float voltage_level_warning_;       // Battery voltage low warning level
    
    float lslope_;                      // Graph slope of the linear speed from joystick input
    float lyintercept_;                 // Graph y-intercept of the linear speed from joystick input
    float aslope_;                      // Graph slope of the angular speed from joystick input
    float ayintercept_;                 // Graph y-intercept of the angular speed from joystick input
    
    const uint16_t SHIFT_CAPS_NUM_LOCK_ = (keyboard::Key::MODIFIER_NUM | keyboard::Key::MODIFIER_CAPS | 
                                           keyboard::Key::MODIFIER_LSHIFT | keyboard::Key::MODIFIER_RSHIFT);
    const unsigned int MAX_AXES_VALUE_ = 32767; 
                
    void keyboardCallBack(const keyboard::Key::ConstPtr& msg);
    void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void completeCallBack(const std_msgs::String::ConstPtr& msg);
    void motorDemandCallBack(const geometry_msgs::Twist::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void remHeartbeatCallback(const std_msgs::Empty::ConstPtr& msg);
    
    geometry_msgs::Twist rampedTwist(geometry_msgs::Twist prev, geometry_msgs::Twist target,
                                     ros::Time time_prev, ros::Time time_now);
    float rampedVel(float velocity_prev, float velocity_target, ros::Time time_prev, ros::Time time_now, float ramp_rate);   
};

#endif // RODNEY_NODE_H_

