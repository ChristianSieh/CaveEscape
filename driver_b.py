#!/usr/bin/env python

# Driver.py

import rospy
import tf
import numpy as np
import math

from nav_msgs.msg import Path
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

def pathCallback(msg):
    global path, map_rcvd
    # flag to inform that path has been received
    map_rcvd = True
    path = msg.poses

def gpsCallback(msg):
    global gpsx, gpsy, gpsth
    
    gpsx = msg.x
    #Flip y since the map is upside down
    gpsy = -msg.y
    gpsth = msg.theta

def publishPose():
    global gpsx, gpsy, gpsth, trajx, trajy, map_rcvd

	#check if we have received our path yet
    if( not map_rcvd ):
	    return

    msg = PoseStamped()
    
    # important!
    msg.header.frame_id = "map"

    # Publish Position (gpsx,gpsy)
    msg.pose.position.x = gpsx
    msg.pose.position.y = -gpsy

    # Publish theta = pi/4
    # We need to convert from euler coordinates to a quaternion. 
    quaternion = tf.transformations.quaternion_from_euler(0, 0, pathth)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    posePub.publish(msg)

def publishVelocity():
    global trajx,trajy,gpsth,gpsx,gpsy,pathth

    # initialize a twist message
    msg = Twist()
    
    msg.linear.x = gpsx
    msg.linear.y = gpsy
    msg.linear.z = gpsth
    
    msg.angular.x = trajx
    msg.angular.y = trajy
    
    if(trajx == 0):
        #determine direction
        #down
        if(trajy < 0):
            msg.angular.z = -np.pi/2.0
        #up
        else:
            msg.angular.z = np.pi/2.0
    else:
        msg.angular.z = math.atan2(trajy, trajx)
        
    # assign our path theta global variable
    pathth = msg.angular.z

    # publish our velocity message
    velPub.publish(msg)

def calculateTrajectory():
    global gpsx,gpsy,path,trajx,trajy,current_path,map_rcvd
	
	#check if we have received our path yet
    if( not map_rcvd ):
	    return
	    
	#check if we have reached our goal of exiting the cave
    if( current_path > (len(path)-1) ):
        trajx = 0
        trajy = 0
        return
    #we haven't reached the end of the cave yet
    #set velocities based on slope equations
    else:
        #calculate distance needed to go
        distx = path[current_path].pose.position.x-gpsx
        disty = (-path[current_path].pose.position.y) - gpsy
        
        trajx = distx
        #need to flip the y trajectory to compensate for map coordinates
        trajy = -disty
        
    #check if we have completed our current path
    if( abs(distx) < 0.1 and
        abs(disty) < 0.1):
        current_path += 1
        print "Finished Path " + str(current_path-1)
        
# initialize all of our global variables

current_path = 1
sleepCounter = 0
trajx = 0
trajy = 0
map_rcvd = False
gpsx = 0
gpsy = 0
gpsth = 0
pathth = 0

# initialize our driver node
rospy.init_node('driver')

# subscribe to the path from planner.py
rospy.Subscriber("path", Path, pathCallback)
# subscribe to the filtered gps data
rospy.Subscriber("gps_filter", Pose2D, gpsCallback)

# publisher nodes for commanded wheel velocities and pose estimates
velPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
posePub = rospy.Publisher("pose_estimate", PoseStamped, queue_size=10)

# 10 hz refresh rates
rate = rospy.Rate(10) #10 hz

while not rospy.is_shutdown():
    # publish the pose at 2 Hz and everything else at 10 hz
    if( sleepCounter == 5 ):
        publishPose()
        sleepCounter = 0
	    
    calculateTrajectory()
    publishVelocity()
    rate.sleep()
    sleepCounter += 1
