#ifndef RODNEY_NODE_H_
#define RODNEY_NODE_H_

#include <ros/ros.h>
#include <keyboard/Key.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

class RodneyNode
{
public:
    RodneyNode(ros::NodeHandle n);
    void sendRegular(void);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher face_status_pub_;    // Topic to publish status to be displayed by RobotFace
    ros::Publisher mission_pub_;        // Topic to start a mission or small task
    ros::Publisher cancel_pub_;         // Topic to cancel a mission
    ros::Subscriber key_sub_;           // Topic for keyboard input
    ros::Subscriber joy_sub_;           // Topic for joystick/game pad input
    ros::Subscriber mission_sub_;       // Topic for mission complete indication   
            
    bool mission_running_;
    bool manual_locomotion_mode_;    
 
    int linear_speed_index_;   // Controller axes index for linear speed
    int angular_speed_index_;  // Controller axes index for angular speed
    int manual_mode_select_;   // Controller button idex for manual mode (teleop)
    
    int dead_zone_;                     // Controller dead zone value    
    
    float linear_set_speed_;            // Current linear set speed for keyboard manual mode locomoton
    float angular_set_speed_;           // Current angular set speed for keboard manual mode locomoton   
    float keyboard_linear_speed_;       // Linear speed from keyboard
    float keyboard_angular_speed_;      // Anular speed from keyboard
    float joystick_linear_speed_;       // Linear speed from joystick/game pad
    float joystick_angular_speed_;      // Anular speed from joystick/game pad
    float max_linear_speed_;            // Max linear speed in manual mode (teleop)
    float max_angular_speed_;           // Max angular speed in manual mode (teleop)
    
    const uint16_t SHIFT_CAPS_NUM_LOCK_ = (keyboard::Key::MODIFIER_NUM | keyboard::Key::MODIFIER_CAPS | 
                                           keyboard::Key::MODIFIER_LSHIFT | keyboard::Key::MODIFIER_RSHIFT);
    const unsigned int MAX_AXES_VALUE_ = 32767; 
                
    void keyboardCallBack(const keyboard::Key::ConstPtr& msg);
    void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);
    void completeCallBack(const std_msgs::String::ConstPtr& msg);    
};

#endif // RODNEY_NODE_H_

