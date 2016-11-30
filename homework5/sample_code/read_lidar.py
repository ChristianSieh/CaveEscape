#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

# Documentation for this message type:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
def callback(msg):
    print "----- New Scan -----"

    # Minimum angle for laser sweep
    print "Min Angle: ", msg.angle_min

    # Max angle for laser sweep
    print "Max Angle: ", msg.angle_max

    # Shortest distance the laser will measure
    print "Min Range: ", msg.range_min

    # Max distance the laser will measure
    print "Max Range: ", msg.range_max

    # The angle increment between range values in the laser scan
    print "Angle Increment: ", msg.angle_increment

    # Print out some range values...
    print "Ranges: "

    # Smallest angle:
    print "\t" + str(msg.angle_min) + " radians: " + str(msg.ranges[0])

    # Middle angle
    index = len(msg.ranges)/2
    angle = msg.angle_min + (index*msg.angle_increment)
    print "\t" + str(angle) + " radians: " + str(msg.ranges[index])

    # Biggest angle
    index = len(msg.ranges)-1
    print "\t" + str(msg.angle_max) + " radians: " + str(msg.ranges[index])



# Start a ROS node.
rospy.init_node('lidar_read_test')
# Subscribe to the laser scan topic
rospy.Subscriber("laser/scan", LaserScan, callback)

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    # Do stuff...
    i = 1+1
