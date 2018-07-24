#include <rodney/rodney_node.h>
#include <std_msgs/Empty.h>

// Constructor 
RodneyNode::RodneyNode(ros::NodeHandle n)
{
    nh_ = n;
    
    // Subscribe to receive keyboard input and mission complete
    key_sub_ = nh_.subscribe("keyboard/keydown", 100, &RodneyNode::keyboardCallBack, this);
    mission_sub_ = nh_.subscribe("/missions/mission_complete", 5, &RodneyNode::completeCallBack, this);

    // Advertise the topics we publish
    mission_pub_ = nh_.advertise<std_msgs::String>("/missions/mission_request", 5);
    cancel_pub_ = nh_.advertise<std_msgs::Empty>("/missions/mission_cancel", 5);
}
//---------------------------------------------------------------------------

void RodneyNode::keyboardCallBack(const keyboard::Key::ConstPtr& msg)
{        
    // Check no modifiers apart from num lock is excepted
    if((msg->modifiers & ~keyboard::Key::MODIFIER_NUM) == 0)
    {
        // Lower case
        if(msg->code == keyboard::Key::KEY_2)
        {            
            // '2', start a complete scan looking for faces (mission 2)
            std_msgs::String mission_msg;
            mission_msg.data = "M2";
            mission_pub_.publish(mission_msg);
                    
            mission_running_ = true;        
        }
        else if(msg->code == keyboard::Key::KEY_c)
        {          
            // Lower case 'c', cancel missions if one is running
            if(mission_running_ == true)
            {
                std_msgs::Empty empty_msg;
                cancel_pub_.publish(empty_msg);
            }        
        }
        else
        {
            ;
        }
    }
}
//---------------------------------------------------------------------------

void RodneyNode::completeCallBack(const std_msgs::String::ConstPtr& msg)
{
    mission_running_ = false;
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

