#include <rodney/rodney_node.h>
#include <algorithm>
#include <actionlib/client/terminal_state.h>

// Constructor 
RodneyNode::RodneyNode(ros::NodeHandle n) : ac_("head_control_node", true)
{
    nh_ = n;
    
    // Subscribe to receive keyboard input
    key_sub_ = nh_.subscribe("keyboard/keydown", 100, &RodneyNode::keyboardCallBack, this);

    ROS_INFO("RodneyNode: Waiting for action server to start");

    // wait for the action server to start
    ac_.waitForServer(); //will wait for infinite time

    ROS_INFO("RodneyNode: Action server started"); 
}
//---------------------------------------------------------------------------

bool RodneyNode::haveWeSeenThisPerson(FaceSeen face_detected)
{
    bool ret_val = true;

    // Is this person already in our list of people seen
    std::list<FaceSeen>::iterator it = std::find_if(seen_list_.begin(), seen_list_.end(),
    boost::bind(&FaceSeen::id, _1) == face_detected.id);

    if(it == seen_list_.end())
    {
        // Not seen before, add to seen list
        seen_list_.insert(it, face_detected);

        ret_val = false;
    }

    return ret_val;
}
//---------------------------------------------------------------------------

void RodneyNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{        
    // Check no modifiers apart from num lock is excepted
    if((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0)
    {
        // Lower case
        if(msg->code == keyboard::Key::KEY_s)
        {            
            // Lower case 's', start a complete scan looking for faces
            // Send a goal to the action
            face_recognition_msgs::scan_for_facesGoal goal;
        
            // Need boost::bind to pass in the 'this' pointer
            ac_.sendGoal(goal,
                boost::bind(&RodneyNode::doneCB, this, _1, _2),
                boost::bind(&RodneyNode::activeCB, this),                
                boost::bind(&RodneyNode::feedbackCB, this, _1));
                    
            scanning_ = true;        
        }
        else if(msg->code == keyboard::Key::KEY_c)
        {          
            // Lower case 'c', cancel scan if one is running
            if(scanning_ == true)
            {
                ac_.cancelGoal();
            }        
        }
        else if(msg->code == keyboard::Key::KEY_x)
        {
            // Lower case 'x', clear the stores names and id's seen if not running
            if(scanning_ == false)
            {                
                seen_list_.clear();
            }                
        }
        else
        {
            ;
        }
    }
}
//---------------------------------------------------------------------------

// Called once when the goal completes
void RodneyNode::doneCB(const actionlib::SimpleClientGoalState& state,
                        const face_recognition_msgs::scan_for_facesResultConstPtr& result)                 
{
    ROS_DEBUG("RodneyNode: Finished in state [%s]", state.toString().c_str());
    scanning_ = false;    

    if(result->detected.ids_detected.size() > 0)
    {  
        for(unsigned long x = 0; x < result->detected.ids_detected.size(); x++)
        {
            FaceSeen face_detected;
            face_detected.id = result->detected.ids_detected[x];
            face_detected.name = result->detected.names_detected[x];

            if(haveWeSeenThisPerson(face_detected) == false)
            {
                // Log we have seen you now!
                ROS_INFO("RodneyNode: Hello %s, how are you?", result->detected.names_detected[x].c_str());
            }
        }            
    }
}
//---------------------------------------------------------------------------

// Called once when the goal becomes active
void RodneyNode::activeCB()
{
    ROS_DEBUG("RodneyNode: Goal just went active");
}
//---------------------------------------------------------------------------

// Called every time feedback is received for the goal
void RodneyNode::feedbackCB(const face_recognition_msgs::scan_for_facesFeedbackConstPtr& feedback)
{
    ROS_DEBUG("Got Feedback percentage complete %f", feedback->progress);    
    
    if(feedback->detected.ids_detected.size() > 0)
    {  
        for(unsigned long x = 0; x < feedback->detected.ids_detected.size(); x++)
        {
            FaceSeen face_detected;
            face_detected.id = feedback->detected.ids_detected[x];
            face_detected.name = feedback->detected.names_detected[x];             

            if(haveWeSeenThisPerson(face_detected) == false)
            {
                // Log we have seen you now!
                ROS_INFO("RodneyNode: Hello %s, how are you?", feedback->detected.names_detected[x].c_str());
            }
        }          
    }
}
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rodney");
    ros::NodeHandle n;    
    RodneyNode rodney_node(n);   
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
    ros::spin();
    return 0;
}

