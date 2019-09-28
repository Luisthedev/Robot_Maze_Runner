#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robot_maze_runner.msg import lidar_msg
from robot_maze_runner.srv import find_wall


def scan_data_callback(msg):
    global impact_front
    global impact_all

    impact_front = msg.lidar_data[0]
    impact_all = msg.lidar_data
def go_forward():
    if impact_front > .4 or impact_front == 0:
        return True
    else:
        return False

scan_sub = rospy.Subscriber('/scan/chunks', lidar_msg, scan_data_callback)

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
turn_to_wall = rospy.ServiceProxy('find_wall', find_wall)
# Declare us to be a node
rospy.init_node('wall_follow')
rospy.wait_for_service('find_wall')
print'server is running'


impact_front = 1
impact_all = [1]
robo_mvmt = Twist()
has_wall = False

while not rospy.is_shutdown():
    print has_wall
    # if we want to use it finds a wall by shortest dit and turns to it
    #if has_wall == False:
        #print 'server is called'
    #    turn_to_wall()
    #    print 'dljchskjdhajksdakjsdhkajdsbfkajsdfjk'
    #    has_wall = True

    print 'out of loop'
    print min(impact_all)
    while go_forward() == True and min(impact_all) >.4:
        robo_mvmt.linear.x = .3
        cmd_vel_pub.publish(robo_mvmt)
        rospy.sleep(1)
    robo_mvmt.linear.x = .0
    cmd_vel_pub.publish(robo_mvmt)
    rospy.sleep(5)

rospy.spin()
