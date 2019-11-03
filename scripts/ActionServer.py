#! /usr/bin/env python2
import rospy
import actionlib
from robot_maze_runner.msg import ActionServerAction, ActionServerGoal, ActionServerResult, ActionServerFeedback
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

current_angle = 0 # initialize global variable to zero
updates_sent = 0
spin_speed = .3
rightTurn = False
leftTurn = False
# this function is called any time MainNode sends an ActionServerGoal to ActionServer
def turn_robot(goal):
    print(str(goal.desired_rotation))
    global current_angle # make sure this variable is global
    global updates_sent
    global spin_speed
    start_angle = current_angle # current_angle should be constantly updated from odometry feedback
    rotate_twist = Twist() # create a Twist object
    # Set the z angular velocity to be the desired_rotation angle in radians. This should cause the robot
    # to complete the turn in 1 second
    if (goal.desired_rotation < 180):
        rotate_twist.angular.z = spin_speed
    else:
        rotate_twist.angular.z = -spin_speed
    
    desired_angle = (start_angle + goal.desired_rotation) % 360
    
    t0 = rospy.Time.now().to_sec() # mark the current time
    # while the current_angle (updated by odometry) is < the desired angle, publish the rotation command
    # to cmd_vel
    rate = rospy.Rate(50)
    acceptable_error = 1
    
    while abs(current_angle - desired_angle) > acceptable_error:
        print("start angle: " + str(start_angle) + " " +  str(current_angle) + " / " + str(desired_angle))
        cmd_vel_pub.publish(rotate_twist)
        tnow = rospy.Time.now().to_sec()
        tdif = tnow - t0 # elapsed time since beginning of action
        feedback = ActionServerFeedback() # create a feedback object
        feedback.degrees_rotated = current_angle # send the # of degrees rotated
        feedback.time_elapsed = tdif # send the time elapsed since action start
        server.publish_feedback(feedback) # publish feedback
        updates_sent+=1 # increment the number of updates sent
        rate.sleep()
    
    rotate_twist.angular.z = 0 # once the while loop completes (action should be done), set the angular velocity to 0
    cmd_vel_pub.publish(rotate_twist)
    result = ActionServerResult() # create an ActionServerResult()
    result.completed = 1 # action completed
    result.updates_sent = updates_sent # send back total number of updates
    server.set_succeeded(result, "Rotation finished") # set that the action succeeded

# callback to handle odometry feedback. Mainly it updates the current angle of the robot
def odom_callback(msg):
    global current_angle
    cur_poseWcov = msg.pose
    cur_pose = cur_poseWcov.pose
    cur_orientation = cur_pose.orientation # extract the quaternion describing robots current rotation
    explicit_quat = [cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w]
    euler_angles = euler_from_quaternion(explicit_quat) # convert from quaternion->radians
    current_angle = euler_angles[2]*360/(2*math.pi) # convert from radians->degrees
    if (current_angle < 0):
        current_angle = 360 + current_angle
    else:
        current_angle = current_angle


rospy.init_node('ActionServer') # create the node
server=actionlib.SimpleActionServer('actionserver',ActionServerAction,turn_robot,False) # create the action server
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) # declare that this node will publish to cmd_vel
odom_sub = rospy.Subscriber('odom', Odometry, odom_callback, queue_size=10) # subscribe to /odom
server.start() # start the server
rospy.spin()
