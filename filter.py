#! /usr/bin/env python

# Filter.py

import rospy
import numpy as np

from geometry_msgs.msg import Pose2D

def gpsCallback(msg):
    global gpsx, gpsy, gpsth
    gpsx = msg.x
    gpsy = msg.y
    gpsth = msg.theta
    
def cmdCallback(msg):
    global phi_1, phi2
    
    #store our latest commanded wheel velocities
    phi_1 = msg.velocities[0]
    phi_2 = msg.velocities[1]
    
def filterGPS():
    global gpsx,gpsy,gpsth,filteredx,filteredy,filteredth,phi_1,phi_2,oldx,oldy,oldth
    
    
    
    
def publishFilteredGPS():
    global filteredx, filteredy, filteredth
    
    #create a new Pose2D object
    msg = Pose2D()
    
    msg.x = filteredx
    msg.y = filteredy
    msg.theta = filteredth
    
    #publish the filtered data
    gpsPub.Publish(msg)
    
    
# create and initialize global variables
gpsx = 0
gpsy = 0
gpsth = 0

phi_1 = 0
phi_2 = 0

filteredx = 0
filteredy = 0
filteredth = 0

xp0 = 0
xp1 = 0
xf0 = 0
xf1 = 0

# Define constants
L = 0.15
r = 0.05

# initialize our node
rospy.init_node('filter')

# create a subscriber to the unfiltered gps and a publisher for
# the filtered gps
gpsPub = rospy.Publisher("gps_filter", Pose2D, queue_size=10)
rospy.Subscriber("gps", Pose2D, gpsCallback)
rospy.Subscriber("cmd_joint_traj", JointTrajectory, cmdCallback)

rate = rospy.Rate(10) #10 hz

while not rospy.is_shutdown():
    #filter the GPS signal
    filterGPS()
    #publish the filtered data
    publishFilteredGPS()
    #sleepy sleep    
    rate.sleep()
