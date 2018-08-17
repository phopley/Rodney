#include <rodney/rodney_node.h>
#include <std_msgs/Empty.h>
#include <ros/package.h>

// Constructor 
RodneyNode::RodneyNode(ros::NodeHandle n)
{
    nh_ = n;
    
    // Subscribe to receive keyboard input and mission complete
    key_sub_ = nh_.subscribe("keyboard/keydown", 100, &RodneyNode::keyboardCallBack, this);
    mission_sub_ = nh_.subscribe("/missions/mission_complete", 5, &RodneyNode::completeCallBack, this);

    // Advertise the topics we publish
    face_status_pub_ = nh_.advertise<std_msgs::String>("/robot_face/expected_input", 5);
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
        else if(msg->code == keyboard::Key::KEY_s)
        {
            // Lower case 's', test status
            
            // For now do this here also
            // RobotFace to centre of screen and bring to the front
            system("wmctrl -r \"RobotFace\" -e 0,200,0,0,0");
            
            std_msgs::String status_msg;            
            status_msg.data = "Rodney";
            face_status_pub_.publish(status_msg);
        }
        else if(msg->code == keyboard::Key::KEY_w)
        {
            // Lower case 'w', test wav playing
            // This is a simple task not a mission
            std_msgs::String mission_msg;
            std::string path = ros::package::getPath("rodney");
            mission_msg.data = "T1-" + path + "/sounds/lost_in_space_danger.wav" + "-Danger Will Robinson danger:)";
            ROS_INFO("%s", path.c_str());
            mission_pub_.publish(mission_msg);            
        }
        else if(msg->code == keyboard::Key::KEY_t)
        {
            // Lower case 't', test speech playing
            // This is a simple task not a mission
            std_msgs::String mission_msg;
            mission_msg.data = "T2-happy birthday u an-Happy birthday Iwan:)";
            mission_pub_.publish(mission_msg);
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

