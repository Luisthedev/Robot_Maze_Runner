#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from robot_maze_runner.msg import clean_lidar


def scan_callback(msg):
    lidar_ms = clean_lidar()
    for i in range(len(msg.ranges)):
        if msg.ranges[i] == 0 or math.isinf(msg.ranges[i]):
            lidar_ms.rangeVals.append(-1)
        else:
            lidar_ms.rangeVals.append(msg.ranges[i])

    pub.publish(lidar_ms)

rospy.init_node('clean_lidar')

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

pub = rospy.Publisher('/scan/clean', clean_lidar, queue_size=1)



print('Clean Lidar has been created')

rospy.spin()
