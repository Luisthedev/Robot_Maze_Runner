#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def scan_data_callback(msg):
    global impact_front
    global impact_all

    impact_front = msg[0]
    impact_all = msg
def go_forward():
    if impact_front > .4 or impact_front == 0:
        return True
    else:
        return False

scan_sub = rospy.Subscriber('/scan/chunks', array[], scan_data_callback)

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Declare us to be a node
rospy.init_node('wall_follow')
impact_front = 1
impact_all = []
robo_mvmt = Twist()
while not rospy.is_shutdown():
    while go_forward == True and min(impact_all) >.4:
        robo_mvmt.linear.x = .3
        cmd_vel_pub.publish(robo_mvmt)
    robo_mvmt.linear.x = .0
    cmd_vel_pub.publish(robo_mvmt)
