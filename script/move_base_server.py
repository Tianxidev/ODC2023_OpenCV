#!/usr/bin/env python
# -*- coding: utf-8 -*-
# path py2.7

'''
@File    :   odc_ap_deep_camera.py
@Time    :   2023-07-21 15:12:20
@Author  :   ZhangBo<seczhangbo@163.com>
@Version :   1.0
@Contact :   移动控制服务端 
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import atan2, sqrt

turtle_target_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
turtle_vel = None

def pose_callback(msg):
    global turtle_original_pose
    # Extract the position and orientation information from the received message
    turtle_original_pose['x'] = msg.pose.pose.position.x
    turtle_original_pose['y'] = msg.pose.pose.position.y
    turtle_original_pose['theta'] = msg.pose.pose.orientation.z
    rospy.loginfo("turtle1_position: (%f, %f, %f)", turtle_original_pose['x'], turtle_original_pose['y'], turtle_original_pose['theta'])

def execute(goal):
    global turtle_target_pose, turtle_original_pose, turtle_vel

    feedback = Pose()

    rospy.loginfo("TurtleMove is working.")
    turtle_target_pose['x'] = goal.turtle_target_x
    turtle_target_pose['y'] = goal.turtle_target_y
    turtle_target_pose['theta'] = goal.turtle_target_theta

    while not rospy.is_shutdown():
        vel_msgs = Twist()
        break_flag = 0.0

        vel_msgs.angular.z = 4.0 * (atan2(turtle_target_pose['y'] - turtle_original_pose['y'],
                                         turtle_target_pose['x'] - turtle_original_pose['x']) -
                                   turtle_original_pose['theta'])
        vel_msgs.linear.x = 0.5 * sqrt(
            pow(turtle_target_pose['x'] - turtle_original_pose['x'], 2) +
            pow(turtle_target_pose['y'] - turtle_original_pose['y'], 2))
        break_flag = sqrt(
            pow(turtle_target_pose['x'] - turtle_original_pose['x'], 2) +
            pow(turtle_target_pose['y'] - turtle_original_pose['y'], 2))
        turtle_vel.publish(vel_msgs)

        feedback.x = turtle_original_pose['x']
        feedback.y = turtle_original_pose['y']
        feedback.theta = turtle_original_pose['theta']
        server.publish_feedback(feedback)
        rospy.loginfo("break_flag=%f", break_flag)
        if break_flag < 0.1:
            break
        rospy.sleep(0.1)

    rospy.loginfo("TurtleMove is finished.")
    server.set_succeeded()

if __name__ == "__main__":
    rospy.init_node("turtleMove")

    # Replace the topic name "/odom" with the actual topic name for the odometry data
    turtle_node = rospy.Subscriber("/odom", Odometry, pose_callback)
    turtle_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    server = actionlib.SimpleActionServer("turtleMove", Odometry, execute, False)
    server.start()

