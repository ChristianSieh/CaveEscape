#!/usr/bin/env python

# Planner.py

import rospy as planner

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np


# Empty Space: 0
# Wall: 100
# Unknown: -1
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

    newMap = np.zeros((msg.info.width * msg.info.height))

    # Increase size of obstacles by .2m since Robie is .17m wide
    for i, value in enumerate(msg.data):
        if(value == 100):
            # Left
            if((i % msg.info.width) - 1 > 0):
                newMap[i - 1] = 100
                if((i % msg.info.width) - 2 > 0):
                    newMap[i - 2] = 100

            # Right
            if((i % msg.info.width) + 1 <  msg.info.width):
                newMap[i + 1] = 100
                if((i % msg.info.width) + 2 < msg.info.width):
                    newMap[i + 2] = 100

            # Up
            if(i - msg.info.height > 0):
                newMap[i - msg.info.width] = 100
                if(i - (2 * msg.info.height) > 0):
                    newMap[i - (2 * msg.info.width)] = 100

            # Down
            if(i + msg.info.height < (msg.info.height * msg.info.width)): 
                newMap[i + msg.info.width] = 100
                if(i + (2 * msg.info.height) < (msg.info.height * msg.info.width)): 
                    newMap[i + (2 * msg.info.width)] = 100

            # Up Left
            if(((i % msg.info.width) - 1 > 0) and (i - msg.info.height > 0)):
                newMap[(i - msg.info.width) - 1] = 100
                if(((i % msg.info.width) - 2 > 0) and (i - (2 * msg.info.height) > 0)):
                    newMap[(i - (2 * msg.info.width)) - 2] = 100

            # Up Right
            if(((i % msg.info.width) + 1 > 0) and (i - msg.info.height > 0)):
                newMap[(i - msg.info.width) + 1] = 100
                if(((i % msg.info.width) + 2 > 0) and (i - (2 * msg.info.height) > 0)):
                    newMap[(i - (2 * msg.info.width)) + 2] = 100

            # Down Left
            if(((i % msg.info.width) - 1 > 0) and (i + msg.info.height < (msg.info.height * msg.info.width))):
                newMap[(i + msg.info.width) - 1] = 100
                if(((i % msg.info.width) - 2 > 0) and (i + (2 * msg.info.height) < (msg.info.height * msg.info.width))):
                    newMap[(i + (2 * msg.info.width)) - 2] = 100

            # Down Right
            if(((i % msg.info.width) + 1 > 0) and (i + msg.info.height < (msg.info.height * msg.info.width))):
                newMap[(i + msg.info.width) + 1] = 100
                if(((i % msg.info.width) + 2 > 0) and (i + (2 * msg.info.height) < (msg.info.height * msg.info.width))):
                    newMap[(i + (2 * msg.info.width)) + 2] = 100

        else:
            newMap[i] = value

    for i, value in enumerate(newMap):
        if(value == -1.0):
            newMap[i] = 127
        if(value == 0.0):
            newMap[i] = 255
        if(value == 100.0):
            newMap[i] = 0

    test = np.reshape(newMap, (200, 200))

    target = open("test", 'w')

    #for i, value in enumerate(newMap):
        #if(i == 200):
        #    target.write("\n")
        #    i = 0
        #else:
        #    target.write(str(value) + " ")

    #target.write(test)

    #plt.gray()
    #plt.imshow(test)

    blah = Image.fromarray(test)
    blah.show()

    print "done"

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
#planner.Subscriber("laser/scan", LaserScan, lidarCallback)

pathPub = planner.Publisher("path", Path, queue_size=10)

rate = planner.Rate(2) # 2 hz
#rate = rospy.Rate(40) # 40 hz

while not planner.is_shutdown():
    #publishPath()
    rate.sleep()




