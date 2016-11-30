#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped

# Start a ROS node.
rospy.init_node('pose_publish_test')
pub = rospy.Publisher("pose_estimate", PoseStamped, queue_size=10)
rate = rospy.Rate(10) # 10 hz

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    msg = PoseStamped()

    # important!
    msg.header.frame_id = "map"

    # Publish Position (1,0)
    msg.pose.position.x = 1.0
    msg.pose.position.y = 0.0

    # Publish theta = pi/4
    # We need to convert from euler coordinates to a quaternion.
    quaternion = tf.transformations.quaternion_from_euler(0,0,3.14/4.0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub.publish(msg)
    rate.sleep()
