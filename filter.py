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
    
def filterGPS():
    global gpsx, gpsy, gpsth, filteredx, filteredy, filteredth
    
    # use a kalman filter to filter our noise
    
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

filteredx = 0
filteredy = 0
filteredth = 0

# initialize our node
rospy.init_node('filter')

# create a subscriber to the unfiltered gps and a publisher for
# the filtered gps
gpsPub = rospy.Publisher("gps_filter", Pose2D, queue_size=10)
rospy.Subscriber("gps", Pose2D, gpsCallback)

rate = rospy.Rate(10) #10 hz

while not rospy.is_shutdown():
    #filter the GPS signal
    filterGPS()
    #publish the filtered data
    publishFilteredGPS()
    #sleepy sleep    
    rate.sleep()
