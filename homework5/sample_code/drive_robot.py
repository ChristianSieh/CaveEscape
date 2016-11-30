#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Start a ROS node.
rospy.init_node('drive_robot_test')
# Wheel velocity publisher
pub = rospy.Publisher("cmd_joint_traj", JointTrajectory, queue_size=10)
# Rate to publish the wheel velocities
rate = rospy.Rate(10) # 10 hz

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    # Create a message to publish
    cmd = JointTrajectory()

    # Add joint names for our left and right wheels
    cmd.joint_names.append("left")
    cmd.joint_names.append("right")

    # Add our wheel velocities (radians/sec)
    p = JointTrajectoryPoint()
    p.velocities.append(0.0) # left wheel
    p.velocities.append(1.0) # right wheel
    cmd.points = [p]

    # Publish our wheel velocities
    pub.publish(cmd)
    rate.sleep()
