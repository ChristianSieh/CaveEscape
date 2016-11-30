#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Start a ROS node.
rospy.init_node('path_publish_test')
pub = rospy.Publisher("path", Path, queue_size=10)
rate = rospy.Rate(10) # 10 hz

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    msg = Path()

    # important!
    msg.header.frame_id = "map"

    p = PoseStamped()
    p.pose.position.x = 0.0
    p.pose.position.y = 0.0
    msg.poses.append(p)

    p = PoseStamped()
    p.pose.position.x = 2.0
    p.pose.position.y = 0.0
    msg.poses.append(p)

    p = PoseStamped()
    p.pose.position.x = 3.25
    p.pose.position.y = 2.0
    msg.poses.append(p)

    pub.publish(msg)
    rate.sleep()
