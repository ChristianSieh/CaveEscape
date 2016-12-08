#!/usr/bin/env python

# Diff_Drive.py

import rospy
import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

def cmd_velCallback(msg):
    global phi_1,phi_2,speed_count
    
    # calculate the difference between our directional theta(dot)
    # and the positional theta
    if(msg.linear.z > (np.pi / 2.0) and msg.angular.z < (-np.pi / 2.0)):
        newTheta = msg.angular.z + np.pi * 2
    else:
        newTheta = msg.angular.z

    theta_dot = newTheta - msg.linear.z
   
    print msg.linear.z, newTheta
    
    # Increment Speed counter
    speed_count += 1
    # calculate our wheels speeds from our theta dot
    if(speed_count % 5 == 0):
        speed_count = 0
        phi_1 = (L*theta_dot)/r+v/r
        phi_2 = -(L*theta_dot)/r+v/r
    
    # if the trajectories are both 0, we are trying
    # to indicate a stop condition from the driver
    if(msg.angular.x == 0 and msg.angular.y == 0):
        phi_1 = 0
        phi_2 = 0
    
    
def publishWheel():
    global phi_1,phi_2
    # Create a message to publish
    cmd = JointTrajectory()

    # Add joint names for our left and right wheels
    cmd.joint_names.append("left")
    cmd.joint_names.append("right")

    # Add our wheel velocities (radians/sec)
    p = JointTrajectoryPoint()
    p.velocities.append(phi_2) # left wheel
    p.velocities.append(phi_1) # right wheel
    cmd.points = [p]

    # Publish our wheel velocities
    wheelPub.publish(cmd)

# Initialize globals
phi_1 = 0
phi_2 = 0

# Count for recalculation of wheel speed
speed_count = 0

# Define constants
L = 0.15
r = 0.05
# speed
v = 0.3

# Start a ROS node.
rospy.init_node('diff_drive')

# Wheel velocity publisher
wheelPub = rospy.Publisher("cmd_joint_traj", JointTrajectory, queue_size=10)
# Subscribe to the command velocity twist messages
rospy.Subscriber("cmd_vel", Twist, cmd_velCallback)

rate = rospy.Rate(10) # 10 hz

while not rospy.is_shutdown():
    publishWheel()
    rate.sleep()
