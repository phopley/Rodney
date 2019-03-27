#!/usr/bin/env python
import sys
import rospy
import tf
import tf2_ros
import geometry_msgs.msg

def main(args):
    rospy.init_node('rodney_static_broadcaster', anonymous=False)
    rospy.loginfo("rodney static broadcaster node started")
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Static transform for the base_footprint to base_link
    st1 = geometry_msgs.msg.TransformStamped()
    st1.header.stamp = rospy.Time.now()
    st1.header.frame_id = "/base_footprint"
    st1.child_frame_id = "/base_link"
    st1.transform.translation.x = 0.0
    st1.transform.translation.y = 0.0
    st1.transform.translation.z = 0.09
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    st1.transform.rotation.x = quat[0]
    st1.transform.rotation.y = quat[1]
    st1.transform.rotation.z = quat[2]
    st1.transform.rotation.w = quat[3]
    
    # Static transform for the baselink to laser
    st2 = geometry_msgs.msg.TransformStamped()
    st2.header.stamp = rospy.Time.now()
    st2.header.frame_id = "/base_link"
    st2.child_frame_id = "/laser"
    st2.transform.translation.x = 0.085
    st2.transform.translation.y = 0.0
    st2.transform.translation.z = 0.107
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    st2.transform.rotation.x = quat[0]
    st2.transform.rotation.y = quat[1]
    st2.transform.rotation.z = quat[2]
    st2.transform.rotation.w = quat[3]

    # Static transform for the baselink to imu
    st3 = geometry_msgs.msg.TransformStamped()
    st3.header.stamp = rospy.Time.now()
    st3.header.frame_id = "/base_link"
    st3.child_frame_id = "/imu"
    st3.transform.translation.x = 0.0
    st3.transform.translation.y = 0.0
    st3.transform.translation.z = 0.058
    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    st3.transform.rotation.x = quat[0]
    st3.transform.rotation.y = quat[1]
    st3.transform.rotation.z = quat[2]
    st3.transform.rotation.w = quat[3]
         
    broadcaster.sendTransform([st1, st2, st3]) 
    
    # static transforms are latched so all we need to do here is spin
    rospy.spin()     

if __name__ == '__main__':
    main(sys.argv)
