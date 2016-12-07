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

goalX = 7.0
goalY = 2.0
goalIndex = 0.0
startX = 0.0
startY = 0.0
startIndex = 0.0
mapWidth = 0
mapHeight = 0
path = list()

# Empty Space: 0
# Wall: 100
# Unknown: -1
def mapCallback(msg):
    global mapWidth, mapHeight, startIndex, goalIndex, path

    print "----- New Map -----"
    print "Width: ", msg.info.width
    print "Height: ", msg.info.height
    print "Resolution: ", msg.info.resolution
    print "Origin: \n", msg.info.origin

    # Calculate goal index
    xindex = int((goalX - msg.info.origin.position.x) * (1.0 / msg.info.resolution))
    yindex = int((goalY - msg.info.origin.position.y) * (1.0 / msg.info.resolution))
    goalIndex = yindex * msg.info.width + xindex
    print "Goal Index: ", goalIndex

    # Calculate start index
    xindex = int((startX - msg.info.origin.position.x) * (1.0 / msg.info.resolution))
    yindex = int((startY - msg.info.origin.position.y) * (1.0 / msg.info.resolution))
    startIndex = yindex * msg.info.width + xindex
    print "Start Index: ", startIndex

    originX = msg.info.origin.position.x
    originY = msg.info.origin.position.y
    resolution = msg.info.resolution

    imageMap = np.zeros((msg.info.width * msg.info.height))

    # Increase size of obstacles by .2m since Robie is .17m wide
    for i, value in enumerate(msg.data):
        if(value == 100):
            imageMap[i] = 100

            # Left
            if((i % msg.info.width) - 1 > 0):
                imageMap[i - 1] = 100
                if((i % msg.info.width) - 2 > 0):
                    imageMap[i - 2] = 100

            # Right
            if(((i % msg.info.width) + 1) <  msg.info.width):
                imageMap[(i + 1)] = 100
                if(((i % msg.info.width) + 2) < msg.info.width):
                    imageMap[(i + 2)] = 100

            # Up
            if(i - msg.info.height > 0):
                imageMap[i - msg.info.width] = 100
                if(i - (2 * msg.info.height) > 0):
                    imageMap[i - (2 * msg.info.width)] = 100

            # Down
            if(i + msg.info.height < (msg.info.height * msg.info.width)): 
                imageMap[i + msg.info.width] = 100
                if(i + (2 * msg.info.height) < (msg.info.height * msg.info.width)): 
                    imageMap[i + (2 * msg.info.width)] = 100

            # Up Left
            if(((i % msg.info.width) - 1 > 0) and (i - msg.info.height > 0)):
                imageMap[(i - msg.info.width) - 1] = 100
                if(((i % msg.info.width) - 2 > 0) and (i - (2 * msg.info.height) > 0)):
                    imageMap[(i - (2 * msg.info.width)) - 2] = 100

            # Up Right
            if(((i % msg.info.width) + 1 < msg.info.width) and (i - msg.info.height > 0)):
                imageMap[(i - msg.info.width) + 1] = 100
                if(((i % msg.info.width) + 2 < msg.info.width) and (i - (2 * msg.info.height) > 0)):
                    imageMap[(i - (2 * msg.info.width)) + 2] = 100

            # Down Left
            if(((i % msg.info.width) - 1 > 0) and (i + msg.info.height < (msg.info.height * msg.info.width))):
                imageMap[(i + msg.info.width) - 1] = 100
                if(((i % msg.info.width) - 2 > 0) and (i + (2 * msg.info.height) < (msg.info.height * msg.info.width))):
                    imageMap[(i + (2 * msg.info.width)) - 2] = 100

            # Down Right
            if((i % msg.info.width) + 1 < msg.info.width and i + msg.info.height < (msg.info.height * msg.info.width)):                          
                imageMap[(i + msg.info.width) + 1] = 100
                if(((i % msg.info.width) + 2 < msg.info.width) and (i + (2 * msg.info.height) < (msg.info.height * msg.info.width))):
                    imageMap[(i + (2 * msg.info.width)) + 2] = 100

        elif(imageMap[i] != 100):
            imageMap[i] = value

    mapWidth = msg.info.width
    mapHeight = msg.info.height

    pathMap = np.zeros((msg.info.width * msg.info.height))

    inc = 1
    path = waveFront(imageMap, pathMap, goalIndex, inc)

    for i, value in enumerate(imageMap):
        if(value == -1.0):
            imageMap[i] = 127
        if(value == 0.0):
            imageMap[i] = 255
        if(value == 100.0):
            imageMap[i] = 0

    for i, value in enumerate(path):
        imageMap[value] = 180

    test = np.reshape(imageMap, (mapWidth, mapHeight))

    blah = Image.fromarray(test)
    blah.show()

def upIndex(index):
    global mapHeight

    if(index - mapHeight > 0):
        upIndex = index - mapHeight
        return upIndex

    return -1

def downIndex(index):
    global mapWidth, mapHeight

    if(index + mapHeight < (mapHeight * mapWidth)):
        downIndex = index + mapHeight
        return downIndex

    return -1

def leftIndex(index):
    global mapWidth, mapHeight    

    if((index % mapWidth) - 1 > 0):
        leftIndex = index - 1
        return leftIndex

    return -1

def rightIndex(index):
    global mapWidth, mapHeight

    if((index % mapWidth) + 1 < mapWidth):
        rightIndex = index + 1
        return rightIndex

    return -1

def upLeftIndex(index):
    global mapWidth, mapHeight

    if(index - mapHeight > 0 and (index % mapWidth) - 1 > 0):
        upLeftIndex = index - mapHeight - 1
        return upLeftIndex

    return -1

def upRightIndex(index):
    global mapWidth, mapHeight

    if(index - mapHeight > 0 and (index % mapWidth) + 1 < mapWidth):
        upRightIndex = index - mapHeight + 1
        return upRightIndex

    return -1

def downLeftIndex(index):
    global mapWidth, mapHeight

    if(index + mapHeight < (mapHeight * mapWidth) and (index % mapWidth) - 1 > 0):
        downLeftIndex = index + mapHeight - 1
        return downLeftIndex

    return -1

def downRightIndex(index):
    global mapWidth, mapHeight

    if(index + mapHeight < (mapHeight * mapWidth) and (index % mapWidth) + 1 < mapWidth):
        downRightIndex = index + mapHeight + 1
        return downRightIndex

    return -1

def waveFront(imageMap, pathMap, index, inc):

    pathMap[index] = inc

    inc += 1

    queue = list()
    queue.append(index)

    while len(queue) != 0:
        current = queue.pop(0)
        for neighbor in getNeighbors(imageMap, current):
            if(pathMap[neighbor] == 0):
                pathMap[neighbor] = inc
                queue.append(neighbor)
        inc += 1

    path = list()

    path = descent(imageMap, pathMap)

    return path

def descent(imageMap, pathMap):
    global startIndex, goalIndex

    tempIndex = startIndex
    path = list()

    # Descent Value, imageMap Index
    neighborDict = dict()

    while tempIndex != goalIndex:
        neighbors = getNeighbors(imageMap, tempIndex)
        
        for i, value in enumerate(neighbors):
            temp = pathMap[value]
            neighborDict[temp] = value
            
        neighborValues = pathMap[neighbors]
        bestNeighbor = min(neighborValues)
        path.append(neighborDict[bestNeighbor])
        tempIndex = neighborDict[bestNeighbor]

    return path    


def getNeighbors(imageMap, index):
    neighbors = list()

    tempIndex = upIndex(index)

    # if the index isn't out of bounds, and it's an open space
    if(tempIndex != -1 and imageMap[tempIndex] == 0):
        neighbors.append(tempIndex)

    tempIndex = rightIndex(index)
    if(tempIndex != -1 and imageMap[tempIndex] == 0):
        neighbors.append(tempIndex)

    tempIndex = downIndex(index)
    if(tempIndex != -1 and imageMap[tempIndex] == 0):
        neighbors.append(tempIndex)

    tempIndex = leftIndex(index)
    if(tempIndex != -1 and imageMap[tempIndex] == 0):
        neighbors.append(tempIndex)

    tempIndex = upLeftIndex(index)
    if(tempIndex != -1 and imageMap[tempIndex] == 0):
        neighbors.append(tempIndex)

    tempIndex = upRightIndex(index)
    if(tempIndex != -1 and imageMap[tempIndex] == 0):
        neighbors.append(tempIndex)

    tempIndex = downLeftIndex(index)
    if(tempIndex != -1 and imageMap[tempIndex] == 0):
        neighbors.append(tempIndex)

    tempIndex = downRightIndex(index)
    if(tempIndex != -1 and imageMap[tempIndex] == 0):
        neighbors.append(tempIndex)    

    return neighbors

#def calculatePath():


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

    newPath = list()

    for i, value in enumerate(path):
        if(i % 5 == 0):
            newPath.append(value)

    for i, value in enumerate(newPath):
        x = value % mapWidth
        y = (value - x) / mapHeight
        x = x / 10.0 - 10
        y = y / 10.0 - 10
        p = PoseStamped()
        p.pose.position.x = x
        p.pose.position.y = -y
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
    publishPath()
    rate.sleep()




