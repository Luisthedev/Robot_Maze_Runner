#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def chunk_callback(msg):
    # get min data in all chuncks
    chunks_array = [6]

    chunks_array[0] = get_min(330,29,msg)
    chunks_array[1] = get_min(30,90,msg)
    chunks_array[2] = get_min(91,151,msg)
    chunks_array[3] = get_min(152,212,msg)
    chunks_array[4] = get_min(213,273,msg)
    chunks_array[5] = get_min(274,329,msg)

    pub.publish(chunks_array)

def get_min(edge_1, edge_2, msg):
    min = sys.maxint
    for i in range(edge_1,edge_2):
        if i != None and i < min:
            min =i
    return min

scan_sub = rospy.Subscriber('/scan/clean', array[], chunk_callback)

pub = rospy.Publisher('/scan/chunks', array[], queue_size=1)

rospy.init_node('chunks')
