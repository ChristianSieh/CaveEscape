#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

###### Functions ######

def mapCallback(msg):
    print "----- New Map -----"
    print "Width: ", msg.info.width
    print "Height: ", msg.info.height
    print "Resolution: ", msg.info.resolution
    print "Origin: \n", msg.info.origin

    # Get the map value at the point (0 meters ,1 meter)
    x = 0.0
    y = 1.0
    # We need to calculate the index into our data array
    xindex = int((x-msg.info.origin.position.x)*(1.0/msg.info.resolution))
    yindex = int((y-msg.info.origin.position.y)*(1.0/msg.info.resolution))
    dataindex = yindex*msg.info.width + xindex
    print "Point (0,1): ", msg.data[dataindex]

    # Do it again... This point has a wall
    x = 0.5
    y = -0.5
    # We need to calculate the index into our data array
    xindex = int((x-msg.info.origin.position.x)*(1.0/msg.info.resolution))
    yindex = int((y-msg.info.origin.position.y)*(1.0/msg.info.resolution))
    dataindex = yindex*msg.info.width + xindex
    print "Point (0.5,-0.5): ", msg.data[dataindex]

    # Do it again... this point is unknown space
    x = 0.0
    y = -1.0
    # We need to calculate the index into our data array
    xindex = int((x-msg.info.origin.position.x)*(1.0/msg.info.resolution))
    yindex = int((y-msg.info.origin.position.y)*(1.0/msg.info.resolution))
    dataindex = yindex*msg.info.width + xindex
    print "Point (0,-1): ", msg.data[dataindex]

def lidarCallback(msg):
    print "----- New Scan -----"

    # Minimum angle for laser sweep
    print "Min Angle: ", msg.angle_min

    # Max angle for laser sweep
    print "Max Angle: ", msg.angle_max

    # Shortest distance the laser will measure
    print "Min Range: ", msg.range_min

    # Max distance the laser will measure
    print "Max Range: ", msg.range_max

    # The angle increment between range values in the laser scan
    print "Angle Increment: ", msg.angle_increment

    # Print out some range values...
    print "Ranges: "

    # Smallest angle:
    print "\t" + str(msg.angle_min) + " radians: " + str(msg.ranges[0])

    # Middle angle
    index = len(msg.ranges)/2
    angle = msg.angle_min + (index*msg.angle_increment)
    print "\t" + str(angle) + " radians: " + str(msg.ranges[index])

    # Biggest angle
    index = len(msg.ranges)-1
    print "\t" + str(msg.angle_max) + " radians: " + str(msg.ranges[index])

def gpsCallback(msg):
    print "----- New Location -----"
    print "X: ", msg.x
    print "Y: ", msg.y
    print "Theta: ", msg.theta

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

def publishPath():
    msg = Path()

    # important!
    msg.header.frame_id = "map"

    p = PoseStamped()
    p.pose.position.x = 0.0
    p.pose.position.y = 0.0
    msg.poses.append(p)

    p = PoseStamped()
    p.pose.position.x = 2.0
    p.pose.position.y = 0.0
    msg.poses.append(p)

    p = PoseStamped()
    p.pose.position.x = 3.25
    p.pose.position.y = 2.0
    msg.poses.append(p)

    pathPub.publish(msg)

def publishPose():
    msg = PoseStamped()

    # important!
    msg.header.frame_id = "map"

    # Publish Position (1,0)
    msg.pose.position.x = 1.0
    msg.pose.position.y = 0.0

    # Publish theta = pi/4
    # We need to convert from euler coordinates to a quaternion.
    quaternion = tf.transformations.quaternion_from_euler(0,0,3.14/4.0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    posePub.publish(msg)

###### MAIN ######

# Start a ROS node.
rospy.init_node('drive_robot_test')

# Wheel velocity publisher
wheelPub = rospy.Publisher("cmd_joint_traj", JointTrajectory, queue_size=10)
pathPub = rospy.Publisher("path", Path, queue_size=10)
posePub = rospy.Publisher("pose_estimate", PoseStamped, queue_size=10)

#Subscribers
rospy.Subscriber("map", OccupancyGrid, mapCallback)
rospy.Subscriber("laser/scan", LaserScan, lidarCallback)
rospy.Subscriber("gps", Pose2D, gpsCallback)

# Rate to publish the wheel velocities
rate = rospy.Rate(40) # 40 hz

sleepCounter = 0;

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    print(sleepCounter)
    if sleepCounter == 20:
        publishPath()
        publishWheel()
        sleepCounter = 0
    rate.sleep()
    sleepCounter += 1
