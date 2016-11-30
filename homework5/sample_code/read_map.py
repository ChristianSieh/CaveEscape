#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

# Documentation for this message type:
# http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html
def callback(msg):
    print "----- New Map -----"
    print "Width: ", msg.info.width
    print "Height: ", msg.info.height
    print "Resolution: ", msg.info.resolution
    print "Origin: \n", msg.info.origin

    # Get the map value at the point (0 meters ,1 meter)
    x = 0.0
    y = 1.0
    # We need to calculate the index into our data array
    xindex = int((x-msg.info.origin.position.x)*(1.0/msg.info.resolution))
    yindex = int((y-msg.info.origin.position.y)*(1.0/msg.info.resolution))
    dataindex = yindex*msg.info.width + xindex
    print "Point (0,1): ", msg.data[dataindex]

    # Do it again... This point has a wall
    x = 0.5
    y = -0.5
    # We need to calculate the index into our data array
    xindex = int((x-msg.info.origin.position.x)*(1.0/msg.info.resolution))
    yindex = int((y-msg.info.origin.position.y)*(1.0/msg.info.resolution))
    dataindex = yindex*msg.info.width + xindex
    print "Point (0.5,-0.5): ", msg.data[dataindex]

    # Do it again... this point is unknown space
    x = 0.0
    y = -1.0
    # We need to calculate the index into our data array
    xindex = int((x-msg.info.origin.position.x)*(1.0/msg.info.resolution))
    yindex = int((y-msg.info.origin.position.y)*(1.0/msg.info.resolution))
    dataindex = yindex*msg.info.width + xindex
    print "Point (0,-1): ", msg.data[dataindex]


# Start a ROS node.
rospy.init_node('map_read_test')
# Subscribe to the map topic
rospy.Subscriber("map", OccupancyGrid, callback)

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    # Do stuff...
    i = 1+1
