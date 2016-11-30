#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D

# Documentation for this message type:
# http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose2D.html
def callback(msg):
    print "----- New Location -----"
    print "X: ", msg.x
    print "Y: ", msg.y
    print "Theta: ", msg.theta

# Start a ROS node.
rospy.init_node('gps_read_test')
# Subscribe to the gps topic
rospy.Subscriber("gps", Pose2D, callback)

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    # Do stuff...
    i = 1+1
