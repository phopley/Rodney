#ifndef RODNEY_NODE_H_
#define RODNEY_NODE_H_

#include <ros/ros.h>
#include <keyboard/Key.h>
#include <std_msgs/String.h>

class RodneyNode
{
public:
    RodneyNode(ros::NodeHandle n);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher face_status_pub_;    // Topic to publish status to be displayed by RobotFace
    ros::Publisher mission_pub_;        // Topic to start a mission or small task
    ros::Publisher cancel_pub_;         // Topic to cancel a mission
    ros::Subscriber key_sub_;           // Topic for keyboard entry
    ros::Subscriber mission_sub_;       // Topic for mission complete indication   
            
    bool mission_running_;        
                
    void keyboardCallBack(const keyboard::Key::ConstPtr& msg);
    void completeCallBack(const std_msgs::String::ConstPtr& msg);    
};

#endif // RODNEY_NODE_H_

