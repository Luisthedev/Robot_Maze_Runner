#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from robot_maze_runner.msg import lidar_msg


def scan_callback(msg):
    lidar_ms = lidar_msg()
    send_ar = lidar_ms.lidar_data
    for i in range(len(msg.ranges)):
        if msg.ranges[i] == 0 or math.isinf(msg.ranges[i]):
            msg.ranges[i]
            send_ar.append(-1)
        else:
            send_ar.append(msg.ranges[i])

    pub.publish(send_ar)

rospy.init_node('clean_lidar')

scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

pub = rospy.Publisher('/scan/clean', lidar_msg, queue_size=1)



print 'Clean Lidar has been created'

rospy.spin()
