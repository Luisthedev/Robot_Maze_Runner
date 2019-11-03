#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from robot_maze_runner.msg import chunks, clean_lidar

def chunk_callback(msg):
    # get min data in all chuncks
    chunks_array = [6]

    msg_ar = msg.rangeVals
    chunk_msg = chunks()
    min_val,max_val = get_min_max1(330,29,msg_ar)
    chunk_msg.min_dists.append(min_val)
    chunk_msg.max_dists.append(max_val)
    min_val,max_val = get_min_max(30,90,msg_ar)
    chunk_msg.min_dists.append(min_val)
    chunk_msg.max_dists.append(max_val)
    min_val,max_val = get_min_max(91,151,msg_ar)
    chunk_msg.min_dists.append(min_val)
    chunk_msg.max_dists.append(max_val)
    min_val,max_val = get_min_max(152,212,msg_ar)
    chunk_msg.min_dists.append(min_val)
    chunk_msg.max_dists.append(max_val)
    min_val,max_val = get_min_max(213,273,msg_ar)
    chunk_msg.min_dists.append(min_val)
    chunk_msg.max_dists.append(max_val)
    min_val,max_val = get_min_max(274,329,msg_ar)
    chunk_msg.min_dists.append(min_val)
    chunk_msg.max_dists.append(max_val)
    
    #print 'this is the chucks array: ' , chunks_array
    pub.publish(chunk_msg)

def get_min_max1(edge_1, edge_2, msg):
    min_val = 10.0
    max_val = 0
    i = 330
    while i <= 359:
        if msg[i] != -1 and msg[i] < min_val:
            min_val = msg[i]
        if msg[i] == -1:
            max_val = float('inf')
        elif msg[i] > max_val:
            max_val = msg[i]
        i=i+1
    i = 0
    while i <=29:
        if msg[i] != -1 and msg[i] < min_val:
            min_val = msg[i]
        if msg[i] == -1:
            max_val = float('inf')
        elif msg[i] > max_val:
            max_val = msg[i]
        i=i+1

    return min_val, max_val

def get_min_max(edge_1, edge_2, msg):
    min_val = 10.0
    max_val = 0
    i = edge_1
    while i <= edge_2:
        #print i
        if msg[i] != -1 and msg[i] < min_val:
            min_val = msg[i]
        if msg[i] == -1:
            max_val = float('inf')
        elif msg[i] > max_val:
            max_val = msg[i]
        i=i+1

    return min_val, max_val

rospy.init_node('chunks')
scan_sub = rospy.Subscriber('/scan/clean', clean_lidar, chunk_callback)

pub = rospy.Publisher('/scan/chunks', chunks, queue_size=1)

print('Chuncks has been created')

rospy.spin()
