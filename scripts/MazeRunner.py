#!/usr/bin/env python
import rospy
from robot_maze_runner.msg import clean_lidar, chunks, ActionServerAction, ActionServerGoal
from geometry_msgs.msg import Twist
import actionlib
import numpy as np


rospy.init_node('MazeRunner')
drive_forward = True
min_vals = []
max_vals = []
minAcceptableDistance = .25
minFrontVal = 1
lastScan = []
"""
#degrees2turn = [0, -60, -120, 180, 120, 60]
degrees2turn = [0, 60, 120, 180, -120, 60]
def chunks_callback(msg):
    global min_vals
    global max_vals
    global drive_forward
    min_vals = msg.min_dists
    max_vals = msg.max_dists
    print(min_vals)
    if (min_vals[0] < minAcceptableDistance):
        drive_forward = False

chunk_sub = rospy.Subscriber('/scan/chunks', chunks,  chunks_callback)
"""
def scan_callback(msg):
    global minMidDistanceVal
    global drive_forward
    global lastScan
    scan = np.asarray(msg.ranges)
    zeroInds = np.where(scan == 0)[0]
    scan[zeroInds.tolist()] = float('inf')
    radsPerSample = msg.angle_increment
    samplesLeftRight int(np.floor(.35/radsPerSample))
    lInd = int(len(scan) - samplesLeftRight)
    lrvals = np.concatentate((scan[lInd:],scan[0:samplesLeftRight+1]))
    minMidDistanceVal = min(lrvals)
    if (minMidDistanceVal < minAcceptableDistance):
        drive_forward = False
    lastScan = scan


cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
client = actionlib.SimpleActionClient('actionserver', ActionServerAction)
client.wait_for_server()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if (drive_forward):
        goForwardCommand = Twist()
        goForwardCommand.linear.x = .4
        cmd_vel_pub.publish(goForwardCommand)
    else:
        stopCommand = Twist()
        stopCommand.linear.x = 0
        cmd_vel_pub.publish(stopCommand)
        furthestChunks = np.argsort(max_vals)
        if (furthestChunks[-1] == 3 or furthestChunks[-1] == 1):
            if (furthestChunks[-2] == 3 or furthestChunks[-2] == 1):
                deg2turn = degrees2turn[furthestChunks[-3]]
            else:
                deg2turn = degrees2turn[furthestChunks[-2]]
        else:
            deg2turn = degrees2turn[furthestChunks[-1]]
        goal = ActionServerGoal()
        goal.desired_rotation = deg2turn
        client.send_goal(goal)
        client.wait_for_result()
        drive_forward = True
        rate.sleep()
rospy.spin()

