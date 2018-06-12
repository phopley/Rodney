#ifndef RODNEY_NODE_H_
#define RODNEY_NODE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition_msgs/scan_for_facesAction.h>
#include <keyboard/Key.h>

struct FaceSeen
{
    unsigned int id;
    std::string name;
};

class RodneyNode
{
public:
    RodneyNode(ros::NodeHandle n);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber key_sub_;
    actionlib::SimpleActionClient<face_recognition_msgs::scan_for_facesAction> ac_;    
    std::list<FaceSeen> seen_list_; // List of faces seen recently
    bool scanning_ = false;
    
    bool haveWeSeenThisPerson(FaceSeen face_detected);
    void keyboardCallBack(const keyboard::Key::ConstPtr& msg);
    void doneCB(const actionlib::SimpleClientGoalState& state,
                const face_recognition_msgs::scan_for_facesResultConstPtr& result);
    void activeCB();
    void feedbackCB(const face_recognition_msgs::scan_for_facesFeedbackConstPtr& feedback);
};

#endif // RODNEY_NODE_H_

