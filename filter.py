#! /usr/bin/env python

# Filter.py

import rospy
import numpy as np
from scipy import linalg
from math import cos,sin

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
    global gpsx,gpsy,gpsth,phi_1,phi_2,xp,sp,xf,P
    
    V = np.array([[0.001,0,0],[0,0.001,0],[0,0,0.001]])
    H = np.array([[1,0,0],[0,1,0],[0,0,1]])
    HT = H.T
    
    v = (r/2.0)*(phi_1+phi_2)
    
    #x prediction = x (filtered??) + time constant * speed in x direction
    xp[0]=xf[0,0]+dt*v*np.cos(xp[2])
    #y prediction = y (filtered??) + time constant * speed in y direction
    xp[1]=xf[1,0]+dt*v*np.sin(xp[2])
    #assign our jacobians with the updated data, we will have a 3x3 of jacobian: x,y,0
    
    
    
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

xp = np.zeros(3)
sp = np.zeros(3)
xf = np.zeros((2,3))
P = np.zeros((2,3,3))

# Define constants
L = 0.15
r = 0.05
dt = 1.0/10.0

# initialize our node
rospy.init_node('filter')

# create a subscriber to the unfiltered gps and a publisher for
# the filtered gps
gpsPub = rospy.Publisher("gps_filter", Pose2D, queue_size=10)
rospy.Subscriber("gps", Pose2D, gpsCallback)
rospy.Subscriber("cmd_joint_traj", JointTrajectory, cmdCallback)

rate = rospy.Rate(10) #10 hz

#publish our initial position of 0
msg = Pose2D()
msg.x = 0
msg.y = 0
msg.theta = 0
gpsPub.Publish(msg)

while not rospy.is_shutdown():
    #filter the GPS signal
    filterGPS()
    #publish the filtered data
    publishFilteredGPS()
    #sleepy sleep    
    rate.sleep()
