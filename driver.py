#!/usr/bin/env python

# Driver.py

import rospy
import tf
import numpy as np

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
    #print "----- New Location -----"
    #print "X: ", msg.x
    #print "Y: ", msg.y
    #print "Theta: ", msg.theta
    gpsx = msg.x
    gpsy = msg.y
    gpsth = msg.theta

def publishPose():
    global gpsx, gpsy, gpsth
    msg = PoseStamped()
    
    # important!
    msg.header.frame_id = "map"

    # Publish Position (gpsx,gpsy)
    msg.pose.position.x = gpsx
    msg.pose.position.y = gpsy

    # Publish theta = pi/4
    # We need to convert from euler coordinates to a quaternion.
    quaternion = tf.transformations.quaternion_from_euler(gpsx,gpsy,gpsth)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    posePub.publish(msg)

def publishVelocity():
    global trajx,trajy,gpsth,gpsx,gpsy 

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
        else:
            msg.angular.z = np.pi/2.0
    else:
        #determine direction
        #first quadrant
        if(trajy > 0 and trajx > 0):
            msg.angular.z = np.arctan(trajy/trajx)
        #second quadrant
        elif(trajy > 0 and trajx < 0):
            msg.angular.z = np.arctan(trajy/abs(trajx)) + np.pi/2.0
        #third quadrant
        elif(trajy < 0 and trajx < 0):
            msg.angular.z = -np.pi/2.0 - np.arctan(abs(trajy)/abs(trajx))
        #fourth quadrant
        else:
            msg.angular.z = -np.arctan(abs(trajy)/trajx)

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
        distx = abs(gpsx - path[current_path].pose.position.x)
        disty = abs(-gpsy - path[current_path].pose.position.y)
        #calculate speeds
        if( disty == 0 ):
            #determine direction
            #left
            if(gpsx < path[current_path].pose.position.x):
                trajx = distx
            #right
            else:
                trajx = -distx
        else:
            #determin direction
            #left
            if(gpsx < path[current_path].pose.position.x):
                trajx = distx
            #right
            else:
                trajx = -distx
        if( distx == 0 ):
            #determine direction
            #down
            if(gpsy < path[current_path].pose.position.y):
                trajy = -disty
            #up
            else:
                trajy = disty
        else:
            #determine direction
            #down
            if(gpsy < path[current_path].pose.position.y):
                trajy = -disty	
            #up
            else:
                trajy = disty
    
        print trajx, trajy
    
    #check if we have completed our current path
    if( distx < 0.2 and
        disty < 0.2):
        current_path += 1
        print "Finished Path " + str(current_path-1)

current_path = 0
sleepCounter = 0
trajx = 0
trajy = 0
map_rcvd = False
gpsx = 0
gpsy = 0
gpsth = 0

rospy.init_node('driver')

rospy.Subscriber("path", Path, pathCallback)
rospy.Subscriber("gps", Pose2D, gpsCallback)

velPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
posePub = rospy.Publisher("pose_estimate", PoseStamped, queue_size=10)

rate = rospy.Rate(10) #10 hz

while not rospy.is_shutdown():
    if( sleepCounter == 5 ):
        publishPose()
        sleepCounter = 0
	    
    calculateTrajectory()
    publishVelocity()
    rate.sleep()
    sleepCounter += 1
