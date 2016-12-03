#!/usr/bin/env python

# Diff_Drive.py

import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def publishWheel():
    # Create a message to publish
    cmd = JointTrajectory()

    # Add joint names for our left and right wheels
    cmd.joint_names.append("left")
    cmd.joint_names.append("right")

    # Add our wheel velocities (radians/sec)
    p = JointTrajectoryPoint()
    p.velocities.append(1.0) # left wheel
    p.velocities.append(1.0) # right wheel
    cmd.points = [p]

    print(p.velocities)

    # Publish our wheel velocities
    wheelPub.publish(cmd)

# Start a ROS node.
rospy.init_node('diff_drive')

# Wheel velocity publisher
wheelPub = rospy.Publisher("cmd_joint_traj", JointTrajectory, queue_size=10)

rate = rospy.Rate(2) # 2 hz

while not rospy.is_shutdown():
    publishWheel()
    rate.sleep()
