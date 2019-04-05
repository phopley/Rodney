/* Copyright 2019 Philip Hopley
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not 
 * use this file except in compliance with the License. You may obtain a  copy
 * of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 *
 * This heartbeat node is not to be run on the robot platform but on a remote 
 * worksation when either the keyboard or joystick nodes are being used to 
 * teleop the robot. If the message sent by this node is missed for 1 second the
 * robot will stop using the keyboard and joystick stored values to drive the motors.
 */
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
