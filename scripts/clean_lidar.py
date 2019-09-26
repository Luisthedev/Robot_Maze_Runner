#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    for i in range(len(msg))
        if msg[i] == 0:
            msg[i] = None
    pub.publish(msg)

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

pub = rospy.Publisher('/scan/clean', array[], queue_size=1)

rospy.init_node('clean_lidar')
