// This heartbeat node is not to be run on the robot platform but on a remote worksation
// when either the keyboard or joystick nodes are being used to teleop the robot. If the
// message sent by this node is missed for 1 second the robot will stop using the keyboard
// and joystick stored values to drive the motors.
#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "remote_heartbeat");
    ros::NodeHandle n;    
    
    ros::Publisher remote_heartbeat_pub = n.advertise<std_msgs::Empty>("remote_heartbeat", 1);
       
    std::string node_name = ros::this_node::getName();
    ROS_INFO("%s started", node_name.c_str());
	
    ros::Rate r(5); // 5Hz	
    
    std_msgs::Empty beat;    
    
    while(ros::ok())
    {
        remote_heartbeat_pub.publish(beat);
        
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;    
}
