#! /usr/bin/env python

# Filter.py

import rospy
import numpy as np
from scipy import linalg
from math import cos,sin
import matplotlib.pyplot as plt

from geometry_msgs.msg import Pose2D
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def gpsCallback(msg):
    global gpsx, gpsy, gpsth
    gpsx = msg.x
    gpsy = msg.y
    gpsth = msg.theta
    
def cmdCallback(msg):
    global phi_1, phi2
    
    #store our latest commanded wheel velocities
    phi_1 = msg.points[0].velocities[0]
    phi_2 = msg.points[0].velocities[1]
    
def filterGPS():
    global gpsx, gpsy, gpsth, phi_1, phi_2, xp, xf, sp, z, filteredx, filteredy, filteredth, P
    
    # Define constants
    L = 0.15
    r = 0.05

    dt = 1.0 / 10.0
    dd = r * dt / 2.0

    H = np.array([[1,0,0],[0,1,0],[0,0,1]]) # Observation Matrix
    HT = H.T
    V = np.array([[0.001,0,0],[0,0.001,0],[0,0,0.001]]) # Process Noise
    W = np.array([[0.04, 0.0, 0.0], [0.0, 0.04, 0.0], [0.0, 0.0, 0.04]]) # Measurement Noise

    #v = (r / 2.0) * (phi_1 + phi_2)
    v = phi_1 + phi_2
    
    # x prediction = x of previous point + time constant * speed in x direction
    xp[0] = xf[0,0] + dd * v * np.cos(xf[0, 2])

    # y prediction = y of previous point + time constant * speed in y direction
    xp[1] = xf[0,1] + dd * v * np.sin(xf[0, 2])

    # theta prediction = theta of previous point + time constant * turn speed ?
    xp[2] = xf[0,2] + dd * (phi_1 - phi_2) / L

    # assign our jacobians with the updated data, we will have a 3x3 of jacobian: x,y,0

    # xf[0, 2] = theta of previous point
    F1 = [1.0, 0.0, -dd * v * cos(xf[0, 2])]
    F2 = [0.0, 1.0, dd * v * sin(xf[0, 2])]
    F = np.array([F1, F2, [0, 0, 1]]) # Process Matrix
    FT = F.T
    pp = np.dot(F, np.dot(P[0], FT)) + V  # P 1|0 covarariance matrix current predition, based on prev??
    z = [gpsx, gpsy, gpsth]
    y = z - np.dot(H, xp) # residual from observation, z is the observed x,y,th
    
    S = np.dot(H, np.dot(pp, HT)) + W
    #S = np.dot(H, np.dot(pp, HT))
    SI = linalg.inv(S)
    kal = np.dot(pp, np.dot(HT, SI)) # Kalman Gain

    # Update previous point to current point
    xf[1] = xp + np.dot(kal,y)
    P[1] = pp - np.dot(kal, pp)

    filteredx, filteredy, filteredth = xf[1]
    
    #update previous values
    xf[0] = xf[1]
    P[0] = P[1]

    #print "XP: ", xp
    #print "XF: ", xf
    #print "V: ", V
    #print "F1: ", F1
    #print "F2: ", F2
    #print "F: ", F
    #print "SI: ", SI
    #print "HT: ", HT
    #print "PP: ", pp
    #print "Kalman Gain: ", kal
    print "gpsx: ", gpsx
    print "gpsy: ", gpsy
    print "gpsth: ", gpsth
    print "filteredx: ", filteredx
    print "filteredy: ", filteredy
    print "filteredth: ", filteredth
    
    
def publishFilteredGPS():
    global filteredx, filteredy, filteredth
    
    #create a new Pose2D object
    msg = Pose2D()
    
    msg.x = filteredx
    msg.y = filteredy
    msg.theta = filteredth
    
    #publish the filtered data
    gpsPub.publish(msg)
    
    
# create and initialize global variables
gpsx = 0
gpsy = 0
gpsth = 0

phi_1 = 0
phi_2 = 0

filteredx = 0
filteredy = 0
filteredth = 0

xf = np.zeros((2, 3))
xp = np.zeros(3)
sp = np.zeros(3)
z = np.zeros(3)
P = np.zeros((2, 3, 3)) # Pose, has to be updated with our pose data?????? pg 226

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
gpsPub.publish(msg)

timearr = []
time = 0

data = []
datagps = []

while not rospy.is_shutdown():
    #filter the GPS signal
    filterGPS()
    #publish the filtered data
    publishFilteredGPS()
    
    #plot data
    timearr.append(time)
    data.append(filteredth)
    datagps.append(gpsth)
    plt.plot(timearr, data, color="blue", linewidth=1.0, linestyle="-")
    plt.plot(timearr, datagps, color="red", linewidth=1.0, linestyle="-")
    plt.ion()
    plt.pause(0.05)
    time += 1
    
    #sleepy sleep    
    rate.sleep()
