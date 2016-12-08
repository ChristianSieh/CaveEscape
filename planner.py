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

    # Calculate goal index
    xindex = int((goalX - msg.info.origin.position.x) * (1.0 / msg.info.resolution))
    yindex = int((goalY - msg.info.origin.position.y) * (1.0 / msg.info.resolution))
    goalIndex = yindex * msg.info.width + xindex

    # Calculate start index
    xindex = int((startX - msg.info.origin.position.x) * (1.0 / msg.info.resolution))
    yindex = int((startY - msg.info.origin.position.y) * (1.0 / msg.info.resolution))
    startIndex = yindex * msg.info.width + xindex

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

    # Calculate Path
    path = waveFront(imageMap, pathMap, goalIndex)

    # Image Drawer

    #for i, value in enumerate(imageMap):
    #    if(value == -1.0):
    #        imageMap[i] = 127
    #    if(value == 0.0):
    #        imageMap[i] = 255
    #    if(value == 100.0):
    #        imageMap[i] = 0

    #for i, value in enumerate(path):
    #    imageMap[value] = 180

    #test = np.reshape(imageMap, (mapWidth, mapHeight))

    #blah = Image.fromarray(test)
    #blah.show()

# Up Index
def upIndex(index):
    global mapHeight

    if(index - mapHeight > 0):
        upIndex = index - mapHeight
        return upIndex

    return -1

# Down Index
def downIndex(index):
    global mapWidth, mapHeight

    if(index + mapHeight < (mapHeight * mapWidth)):
        downIndex = index + mapHeight
        return downIndex

    return -1

# Left Index
def leftIndex(index):
    global mapWidth, mapHeight    

    if((index % mapWidth) - 1 > 0):
        leftIndex = index - 1
        return leftIndex

    return -1

# Right Index
def rightIndex(index):
    global mapWidth, mapHeight

    if((index % mapWidth) + 1 < mapWidth):
        rightIndex = index + 1
        return rightIndex

    return -1

# Up Left Index
def upLeftIndex(index):
    global mapWidth, mapHeight

    if(index - mapHeight > 0 and (index % mapWidth) - 1 > 0):
        upLeftIndex = index - mapHeight - 1
        return upLeftIndex

    return -1

# Up Right Index
def upRightIndex(index):
    global mapWidth, mapHeight

    if(index - mapHeight > 0 and (index % mapWidth) + 1 < mapWidth):
        upRightIndex = index - mapHeight + 1
        return upRightIndex

    return -1

# Down Left Index
def downLeftIndex(index):
    global mapWidth, mapHeight

    if(index + mapHeight < (mapHeight * mapWidth) and (index % mapWidth) - 1 > 0):
        downLeftIndex = index + mapHeight - 1
        return downLeftIndex

    return -1

# Down Right Index
def downRightIndex(index):
    global mapWidth, mapHeight

    if(index + mapHeight < (mapHeight * mapWidth) and (index % mapWidth) + 1 < mapWidth):
        downRightIndex = index + mapHeight + 1
        return downRightIndex

    return -1

def waveFront(imageMap, pathMap, index):

    inc = 1

    pathMap[index] = inc

    inc += 1

    queue = list()
    queue.append(index)

    # BFS using pathMap to keep track of visited
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

    # [Descent Value, imageMap Index]
    neighborDict = dict()

    while tempIndex != goalIndex:
        neighbors = getNeighbors(imageMap, tempIndex)
        
        # Keep the indices and their values together in a dictionary
        for i, value in enumerate(neighbors):
            temp = pathMap[value]
            neighborDict[temp] = value
            
        # Get the descent value of each neighbor        
        neighborValues = pathMap[neighbors]
        # Find the best neighbor
        bestNeighbor = min(neighborValues)
        # Add to path
        path.append(neighborDict[bestNeighbor])
        # Update index
        tempIndex = neighborDict[bestNeighbor]

    return path    

# Get 8 way open neighbors
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

def gpsCallback(msg):
    print "----- New Location -----"
    print "X: ", msg.x
    print "Y: ", msg.y
    print "Theta: ", msg.theta

def publishPath():
    msg = Path()

    # important!
    msg.header.frame_id = "map"

    newPath = list()

    # Only publish every 5th point since we can afford to skip some
    for i, value in enumerate(path):
        if(i % 5 == 0):
            newPath.append(value)

    for i, value in enumerate(newPath):
        # Convert indices to x and y in meters
        x = value % mapWidth
        y = (value - x) / mapHeight
        x = x / 10.0 - 10
        y = y / 10.0 - 10

        # Add to message
        p = PoseStamped()
        p.pose.position.x = x
        p.pose.position.y = y
        msg.poses.append(p)

    pathPub.publish(msg)

planner.init_node('planner')

planner.Subscriber("map", OccupancyGrid, mapCallback)

pathPub = planner.Publisher("path", Path, queue_size=10)

rate = planner.Rate(2) # 2 hz

while not planner.is_shutdown():
    publishPath()
    rate.sleep()




