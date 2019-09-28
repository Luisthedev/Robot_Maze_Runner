#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from robot_maze_runner.msg import lidar_msg
from robot_maze_runner.srv import find_wall
import math


def scan_data_callback(msg):
    global data
    data = msg.lidar_data

def find_wall_callback(msg):
    target = 1
    r = rospy.Rate(10)
    min_dist_dir = get_min_dist(data)
    target = min_dist_dir_degree_heading(min_dist_dir)
    twist = Twist()
    print target
    target_rad = target * math.pi/180
    print yaw
    print target_rad

    while yaw < target_rad :
        print 'yaw in loop',yaw
        print target_rad

        twist.angular.z = .2
        cmd_vel_pub.publish(twist)
        r.sleep()
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)
    print 'put of loop'
    return 0.0


def min_dist_dir_degree_heading(min_dist_dir):
    x = min_dist_dir[1]
    print x
    if x == 'for':
        return 0
    elif x == 'front_right':
        return 90
    elif x == 'back_right':
        return 150
    elif x == 'back':
        return 180
    elif x == 'back_left':
        return 210
    elif x == 'front_left':
        return 270


def newOdom_callback(msg):
    global x
    global y
    global yaw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])



def get_min_dist(data):
    dit_ar = []
    index = 0
    min = 10.0
    for i in range(len(data)):
        if data[i] < min:
            print i
            min = data[i]
            print min
            index = index + 1
    dit_ar.append(min)
    dit_ar.append(get_dir(index))
    print data
    print min
    print 'index=====', index
    return dit_ar

def get_dir(index):
    if index == 0:
        return 'for'
    elif index == 1:
        return 'front_right'
    elif index == 2:
        return 'back_right'
    elif index == 3:
        return 'back'
    elif index == 4:
        return 'back_left'
    elif index == 5:
        return 'front_left'

x = y = yaw = 0.0
rospy.init_node('find_wall')
scan_sub = rospy.Subscriber('/scan/chunks', lidar_msg, scan_data_callback)
sub = rospy.Subscriber("/odom", Odometry, newOdom_callback)
service = rospy.Service("find_wall",find_wall, find_wall_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

data = [1]

rospy.spin()
