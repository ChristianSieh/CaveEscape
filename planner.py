#!/usr/bin/env python

# Planner.py

import rospy as planner

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

def mapCallback(msg):
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

def gpsCallback(msg):
    print "----- New Location -----"
    print "X: ", msg.x
    print "Y: ", msg.y
    print "Theta: ", msg.theta

### Lidar stored here until we know where to put it
def lidarCallback(msg):
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

def publishPath():
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

    pathPub.publish(msg)

planner.init_node('planner')

planner.Subscriber("map", OccupancyGrid, mapCallback)
#planner.Subscriber("gps", Pose2D, gpsCallback)
planner.Subscriber("laser/scan", LaserScan, lidarCallback)

pathPub = planner.Publisher("path", Path, queue_size=10)

rate = planner.Rate(2) # 2 hz
#rate = rospy.Rate(40) # 40 hz

while not planner.is_shutdown():
    publishPath()
    rate.sleep()




