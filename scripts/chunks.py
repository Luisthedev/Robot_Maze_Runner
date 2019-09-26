#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from robot_maze_runner.msg import lidar_msg

def chunk_callback(msg):
    # get min data in all chuncks
    chunks_array = [6]

    msg_ar = msg.lidar_data
    print len(msg_ar)
    print len(chunks_array)

    chunks_array[0] = get_min1(330,29,msg_ar)
    chunks_array.append(get_min(30,90,msg_ar))
    chunks_array.append(get_min(91,151,msg_ar))
    chunks_array.append(get_min(152,212,msg_ar))
    chunks_array.append(get_min(213,273,msg_ar))
    chunks_array.append(get_min(274,329,msg_ar))
    #chunks_array[2] = get_min(91,151,msg_ar)
    #chunks_array[3] = get_min(152,212,msg_ar)
    #chunks_array[4] = get_min(213,273,msg_ar)
    #chunks_array[5] = get_min(274,329,msg_ar)

    #print 'this is the chucks array: ' , chunks_array
    pub.publish(chunks_array)

def get_min1(edge_1, edge_2, msg):
    min = 10.0
    i = 330
    while i <= 359:
        if i != -1 and i < min:
            min = msg[i]
        i=i+1
    i = 0
    while i <=29:
        if msg[i] != -1 and msg[i] < min:
            min = msg[i]
        i=i+1

    return min
def get_min(edge_1, edge_2, msg):
    min = 10.0
    i = edge_1
    while i <= edge_2:
        #print i
        if msg[i] != -1 and msg[i] < min:
            min = msg[i]
        i=i+1

    return min

rospy.init_node('chunks')
scan_sub = rospy.Subscriber('/scan/clean', lidar_msg, chunk_callback)

pub = rospy.Publisher('/scan/chunks', lidar_msg, queue_size=1)



print 'Chuncks has been created'

rospy.spin()
