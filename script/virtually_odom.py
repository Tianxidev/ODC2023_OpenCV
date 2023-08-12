#!/usr/bin/env python
# -*- coding: utf-8 -*-
# path py2.7

'''
@File    :   rviz_map_test.py
@Time    :   2023-07-23 16:41:47
@Author  :   ZhangBo<seczhangbo@163.com>
@Version :   0.1
@Contact :   虚拟里程计
'''

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from math import sin, cos

# 初始化全局变量，用于保存控制指令
vx, vy, vth = 0.0, 0.0, 0.0

# 标记是否有控制指令在进行
is_controlling = False

# Twist消息的回调函数，用于接收控制指令并更新速度
def cmd_vel_callback(msg):
    global vx, vy, vth, is_controlling
    vx = msg.linear.x
    vy = msg.linear.y
    vth = msg.angular.z

    # 标记是否有控制指令在进行
    is_controlling = True

def main():
    global is_controlling  # 声明is_controlling为全局变量
    rospy.init_node("virtually_odom")
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    x, y, th = 0.0, 0.0, 0.0

    # 订阅/cmd_vel主题，指定回调函数为cmd_vel_callback
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(50)
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        # 检查是否有控制指令在进行
        if is_controlling:
            # 根据设置的速度更新里程信息
            dt = (current_time - last_time).to_sec()
            delta_x = (vx * cos(th) - vy * sin(th)) * dt
            delta_y = (vx * sin(th) + vy * cos(th)) * dt
            delta_th = vth * dt

            x += delta_x
            y += delta_y
            th += delta_th

            # 重置控制标志
            is_controlling = False

        # 创建一个TransformStamped消息，通过 tf 发布从“odom”到“base_link”的转换
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # 发布base_link的坐标变换
        odom_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        # 填充Odometry消息
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # 设置位置
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # 设置速度
        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = vx if is_controlling else 0.0
        odom.twist.twist.linear.y = vy if is_controlling else 0.0
        odom.twist.twist.angular.z = vth if is_controlling else 0.0

        # 发布Odometry消息
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo("正在启动虚拟里程计")
        main()
    except rospy.ROSInterruptException:
        pass
