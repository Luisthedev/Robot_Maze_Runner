#!/usr/bin/env python2
import rospy
from robot_maze_runner.msg import clean_lidar, chunks, ActionServerAction, ActionServerGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import actionlib
import numpy as np
import math


rospy.init_node('MazeRunner2')
drive_forward = True
forwardVel = .1
min_vals = []
max_vals = []
minMidDistanceVal = 0
minAcceptableDistance = .3
minFrontVal = 1
lastScan = []
deg2turn = 0
furthestDist = 0
ignoreStart = 2.61
ignoreEnd = 3.66
def scan_callback(msg):
    global minMidDistanceVal
    global drive_forward
    global lastScan
    global deg2turn
    global furthestDist
    scan = np.asarray(msg.ranges)
    zeroInds = np.where(scan == 0)[0]
    scan[zeroInds.tolist()] = float('inf')
    radsPerSample = msg.angle_increment
    samplesLeftRight = int(np.floor(.35/radsPerSample))
    lInd = int(len(scan) - samplesLeftRight)
    lrvals = np.concatenate((scan[lInd:],scan[0:samplesLeftRight+1]))
    minMidDistanceVal = min(lrvals)
    if (minMidDistanceVal < minAcceptableDistance):
        drive_forward = False

    ignoreStartInd = int(round(2.61*(1/radsPerSample)))
    ignoreEndInd = int(round(3.66*(1/radsPerSample)))
    ignoreInds = np.arange(ignoreStartInd,ignoreEndInd+1)
    nonInfInds = np.where(scan != float('inf'))[0]
    good_inds = np.setdiff1d(nonInfInds,ignoreInds)
    maxRangeInds = good_inds[np.argwhere(scan[good_inds] == max(scan[good_inds]))]
    bearingsRad = np.zeros(len(maxRangeInds))
    bearingsDeg = np.zeros(len(maxRangeInds))
    bearingsScores = np.zeros(len(maxRangeInds))
    for idx,val in enumerate(maxRangeInds):
        bearingsRad[idx] = msg.angle_min + val*((msg.angle_max-msg.angle_min)/(len(msg.ranges)-1))
        bearingsDeg[idx] = bearingsRad[idx]*(360/(2*math.pi))
        bearingsScores[idx] = min(abs(bearingsDeg[idx] - 90),abs(bearingsDeg[idx] - 270))
    bestScoreInd = np.argmin(bearingsScores)
    deg2turn = bearingsDeg[bestScoreInd]
    furthestDist = scan[maxRangeInds[bestScoreInd]]
    #print(str(bearingsScores))
    #print(deg2turn)
    #print(str(furthestDist) + '\n')
    #if (deg2turn < 180):
    #    deg2turn = -deg2turn
    #else:
    #    deg2turn = 360 - deg2turn

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
client = actionlib.SimpleActionClient('actionserver', ActionServerAction)
client.wait_for_server()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    #print(str(minMidDistanceVal))
    #print(drive_forward)
    #print(str(deg2turn))
    #print(str(furthestDist))
    #print(str(minMidDistanceVal))
    if (drive_forward):
        goForwardCommand = Twist()
        goForwardCommand.linear.x = forwardVel
        cmd_vel_pub.publish(goForwardCommand)
    else:
        stopCommand = Twist()
        stopCommand.linear.x = 0
        cmd_vel_pub.publish(stopCommand)

        goal = ActionServerGoal()
        goal.desired_rotation = deg2turn
        client.send_goal(goal)
        client.wait_for_result()
        drive_forward = True
    rate.sleep()
rospy.spin()

