#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import socket
import rospy
from std_msgs.msg import String
import threading

# 宝藏真假状态
_TH_ACK = None
pub = None

def callback1(msg):

    if _TH_ACK is not None:
        # 发布数据到指定订阅
        pub.publish(_TH_ACK)

def handle_client(client_socket):
    request = client_socket.recv(1024)

    # 检查是否是宝藏识别消息
    if "TH-" in request:
        rospy.loginfo("接收到宝藏真数据: %s", request)
        ack = request.split('-')[-1]
        _TH_ACK = ack

    else:
        rospy.loginfo("接收到宝藏坐标数据: %s", request)

        with open('/home/bingda/catkin_ws/src/robot_line/script/path.txt','w') as f:    # 设置文件对象
            f.write(request) # 将字符串写入文件中
            f.close()

        # 发布数据到指定订阅
        pub.publish(request)

    client_socket.close()

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 8888))
    server_socket.listen(5)

    rospy.loginfo('Socket 服务已启动, 监听在端口: 8888...')

    while True:
        client_socket, addr = server_socket.accept()
        rospy.loginfo('检测到客户端-> %s 链接', addr)

        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()


if __name__ == "__main__":
    rospy.init_node('tcptalker',anonymous=0)

    # 订阅消息查询
    rospy.Subscriber("/image_raw", String, callback1) 

    # TCP 消息发布
    pub = rospy.Publisher('/tcp_msg_topic', String, queue_size=10)
    start_server()
