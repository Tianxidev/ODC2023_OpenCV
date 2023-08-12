# -*- coding: utf-8 -*-

'''
@File    :   odc_pc.py
@Time    :   2023-08-07 10:02:02
@Author  :   ZhangBo<seczhangbo@163.com>
@Version :   0.1
@Contact :   光电宝藏识别 PC 端
'''

import time
import cv2
import sys
import heapq
import socket
import datetime
import numpy as np
from queue import Queue
from threading import Thread
import tkinter as tk
from PIL import Image, ImageTk


_MAP_CLOCK       = False         # 藏宝图锁
_SEND_DATA_CLOCK = False         # 发送数据锁
_FRAME           = None          # 渲染画面
_BASE_ROS        = "192.168.9.1" # ROS IP 地址


class RosSocket():
    """
    Ros socket通讯
    """

    def __init__(self,HOST,PORT):
        self.BUFFER=4096
        # 定义socket通信类型 ipv4，tcp
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 连接服务器
        try:
            self.soc.connect((HOST, PORT))
            print("成功连接 ROS !")
        except:
            print("连接 ROS 不成功")
        # 延时
        time.sleep(1)

    def send(self, msg):
        try:
            self.soc.send(msg.encode("utf-8"))
            print("成功发送 ROS 消息！")
        except:
            print("发送数据失败")

    def receive(self):
        # 接受消息
        buf = self.soc.recv(self.BUFFER)
        buf.decode("utf-8")
        # 打印消息
        print(buf)
        return buf

    def close(self):
        # 接口关闭
        self.soc.close()

class Node():
    """
    Astar 节点类
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class MazeAstar:
    """
    迷宫寻宝解算
    """

    def __init__(self):

        # 基础迷宫
        self.maze_base = np.array([
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
            [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1],
            [1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
            [1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1],
            [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
            [1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1],
            [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
            [1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1],
            [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        ])


    
    def find_treasures(self, maze):
        """
        寻找路线
        """

        treasures = []
        for i in range(len(maze)):
            for j in range(len(maze[i])):
                if maze[i][j] == 5:
                    treasures.append((i, j))
        return treasures

    def a_star(self, maze, start, end):
        """
        A start 路劲规划实现
        """

        # 创建起始节点和结束节点
        start_node = Node(None, start)
        end_node = Node(None, end)

        # 初始化 open 表和 closed 表
        open_list = []
        closed_list = []

        # 添加起始节点
        heapq.heappush(open_list, (start_node.f, id(start_node), start_node))

        # 持续寻找终点
        while len(open_list) > 0:

            # 获取当前节点
            current_node = heapq.heappop(open_list)[2]
            closed_list.append(current_node)

            # 判断是否找到目标
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] # 反向路径返回

            # 子项生成
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # 相邻方格

                # 获取节点位置
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # 检查是否在范围内
                if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                    continue

                # 检查是否可以行走
                if maze[node_position[0]][node_position[1]] != 0:
                    continue

                # 创建新节点
                new_node = Node(current_node, node_position)

                # 添加节点到子项
                children.append(new_node)

            # 循环遍历子项
            for child in children:
                # 检查是否在 close 表
                for closed_child in closed_list:
                    if child == closed_child:
                        break
                else:
                    # 创建 f, g, h 值
                    child.g = current_node.g + 1
                    child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                    child.f = child.g + child.h

                    # 检查是否已在 open 表中
                    for open_node in open_list:
                        if child == open_node[2] and child.g > open_node[2].g:
                            break
                    else:
                        # 添加到 open 表
                        heapq.heappush(open_list, (child.f, id(child), child))

        return None # 找不到路径
    

    def solve_maze(self, maze, start, end):
        """
        寻路迷宫
        """

        # 寻找所有宝藏
        treasures = self.find_treasures(maze)
        treasures.append(end)
        # print("宝藏列表: ", treasures)
        path = []

        # 路径规划
        for treasure in treasures:
            result = self.a_star(self.maze_base, start, treasure)
            if result is None:
                return None
            # 添加路劲
            path.extend(result)
            start = treasure
        
        # 过滤墙
        router = []
        for x, y in path:
            
            # 构建坐标
            r = (x, y)
            
            # 判断当前 Y 方向是否是墙
            if y % 2 == 0 or x % 2 == 0:
                continue

            router.append(r)

        # 返回路线
        return router

 
class Camera:
    """
    摄像机读取类
    """

    def __init__(self, frame_queue, device_id = None):

        if device_id is None:
            self.device_id = self.gstreamer_pipeline()
        else:
            self.device_id = device_id  # 摄像头id

        # 配置摄像头参数
        self.cam = cv2.VideoCapture(self.device_id)  # 获取摄像头
        self.cam.set(cv2.CAP_PROP_FPS, 30)  # 帧率 帧/秒
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.frame_queue = frame_queue  # 帧队列
        self.is_running = False  # 状态标签
        self.fps = 0.0  # 实时帧率
        self._t_last = time.time() * 1000
        self._data = {}
    
    def capture_queue(self):

        # 捕获图像
        self._t_last = time.time() * 1000
        while self.is_running and self.cam.isOpened():
            ret, frame = self.cam.read()
            if not ret:
                sys.exit(0)
            if self.frame_queue.qsize() < 1: 
                # 当队列中的图像都被消耗完后，再压入新的图像              
                t  = time.time() * 1000
                t_span = t - self._t_last                
                self.fps = int(1000.0 / t_span)
                self._data["image"] = frame.copy()
                self._data["fps"] = int(self.fps)
                self.frame_queue.put(self._data)
                self._t_last = t
 
    def run(self):
        self.is_running = True
        self.thread_capture = Thread(target=self.capture_queue)
        self.thread_capture.start()
 
    def stop(self):
        self.is_running = False
        self.cam.release()

class ScreenProcessing:
    """
    画面处理
    """

    def __init__(self, frame):

        self.ptsFrame = None # 矫正画幅
        self.FrameInfo = None # 画幅大小及通道信息

        # 移动路径
        self.movebase_router = None
        
        # 获取 CV2 版本
        (self.major_ver, self.minor_ver, self.subminor_ver) = (cv2.__version__).split('.')

        # 基础迷宫
        self.t_maze = np.array([
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
            [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1],
            [1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
            [1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1],
            [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1],
            [1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1],
            [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
            [1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1],
            [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        ])

        # 处理图像
        self.handler_frame(frame)

    def drawCross(self, frame, locate, cross_radius=10, color=(0, 255, 255)):
        """
        绘制十字光标
        """

        center_x, center_y = locate
        cross_x1 = center_x - cross_radius
        cross_x2 = center_x + cross_radius
        cross_y1 = center_y - cross_radius
        cross_y2 = center_y + cross_radius
        line1 = np.array([[cross_x1, center_y], [cross_x2, center_y]], np.int32).reshape((-1, 1, 2))
        line2 = np.array([[center_x, cross_y1], [center_x, cross_y2]], np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, pts=[line1, line2], isClosed=False, color=color, thickness=2, lineType=cv2.LINE_8)

    def detectedContours(self, frame):
        """
        画幅轮廓提取
        """
        
        gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
        _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU + cv2.THRESH_BINARY_INV)

        if int(self.major_ver) <= 3:
            _, contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        else:
            contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        return contours, hierarchy

    def checkFrameInHSV(self, frame, lower, higher):
        """
        检查画幅中是否存在指定 HSV 空间
        """

        img_hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)

        # HSV 色彩空间过滤指定区域
        mask = cv2.inRange(img_hsv, lower, higher)
        if int(self.major_ver) <= 3:
            _, cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        else:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        return cnts
    
    def queryPos(self, idx, size=21):
        """
        查询宝藏对应的坐标位置
        :param size: 地图尺寸 默认:21*21
        :param idx: 宝藏方格 大小: 10 * 10 范围: 1-100
        :return: 地图数组坐标
        """

        # 范围 [1, 10)
        if idx in range(1, 10):
            return size - 2, (size - 1) - (idx + idx - 1)

        # 范围 [10, 20)
        if idx in range(10, 20):
            if idx == 10:
                return size - 2, 1
            idx = idx % 10
            return size - 4, (size - 1) - (idx + idx - 1)

        # 范围 [20, 30)
        if idx in range(20, 30):
            if idx == 20:
                return size - 4, 1
            idx = idx % 10
            return size - 6, (size - 1) - (idx + idx - 1)

        # 范围 [30, 40)
        if idx in range(30, 40):
            if idx == 30:
                return size - 6, 1
            idx = idx % 10
            return size - 8, (size - 1) - (idx + idx - 1)

        # 范围 [40, 50)
        if idx in range(40, 50):
            if idx == 40:
                return size - 8, 1
            idx = idx % 10
            return size - 10, (size - 1) - (idx + idx - 1)

        # 范围 [50, 60)
        if idx in range(50, 60):
            if idx == 50:
                return size - 10, 1
            idx = idx % 10
            return size - 12, (size - 1) - (idx + idx - 1)

        # 范围 [60, 70)
        if idx in range(60, 70):
            if idx == 60:
                return size - 12, 1
            idx = idx % 10
            return size - 14, (size - 1) - (idx + idx - 1)

        # 范围 [70, 80)
        if idx in range(70, 80):
            if idx == 70:
                return size - 14, 1
            idx = idx % 10
            return size - 16, (size - 1) - (idx + idx - 1)

        # 范围 [80, 90)
        if idx in range(80, 90):
            if idx == 80:
                return size - 16, 1
            idx = idx % 10
            return size - 18, (size - 1) - (idx + idx - 1)

        # 范围 [90, 100)
        if idx in range(90, 100):
            if idx == 90:
                return size - 18, 1
            idx = idx % 10
            return size - 20, (size - 1) - (idx + idx - 1)

        # 范围 idx == 100
        if idx == 100:
            return 1, 1

        return None
    


    def drawGrid(self, frame, grid_shape, color=(0, 255, 0), thinkness=2):
        """
        绘制网格
        """

        h, w, _ = frame.shape
        rows, cols = grid_shape
        dy, dx = h / rows, w / cols
        for x in np.linspace(start=dx, stop=w - dx, num=cols - 1):
            x = int(round(x))
            cv2.line(frame, (x, 0), (x, h), color=color, thickness=thinkness)
        for y in np.linspace(start=dy, stop=h - dy, num=rows - 1):
            y = int(round(y))
            cv2.line(frame, (0, y), (w, y), color=color, thickness=thinkness)
        return frame

    def calcFrameCenter(self, x, y, w, h):
        """
        计算画幅中心
        """

        x = x + w // 2
        y = y + h // 2

        return x, y

    def SpeckleFind(self, frame):
        """
        寻找斑点
        """

        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 100
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(frame)
        blank = np.zeros((1, 1))
        color = (0, 255, 0) # 绿色
        blobs = cv2.drawKeypoints(frame, keypoints, blank, color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        blobs = cv2.drawKeypoints(blobs, keypoints, blank, color, cv2.DRAW_MATCHES_FLAGS_DEFAULT)

        # 获取每个特征点的坐标
        keypoint_coordinates = []
        for kp in keypoints:
            x, y = kp.pt
            keypoint_coordinates.append((int(x), int(y)))

        return blobs, keypoint_coordinates
    
    def detect_edges(self, image, min_value, max_value):
        """
        Canny 轮廓提取
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, min_value, max_value)
        return edges

    def ScreenCorrection(self, frame, pts_input, w=600, h=600):
        """
        逆透视变换
        """
        pts_d = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
        M = cv2.getPerspectiveTransform(pts_input, pts_d)
        return cv2.warpPerspective(frame, M, (w, h))

    def findMapArea(self, frame):
        """
        寻找地图区域
        """

        contours, _ = self.detectedContours(frame.copy())
        for cont in contours:
            epsilon = 0.01 * cv2.arcLength(cont, True)
            approx = cv2.approxPolyDP(cont, epsilon, True)
            if len(approx) != 8:
                continue
            x, y, w, h = cv2.boundingRect(cont)
            new_frame = frame[y:(y + h), x:(x + w)]
            new_frame = cv2.resize(new_frame, (700, 600))
            return new_frame
        return None

    def findMapTreasureGrid(self, frame, keypoint_coordinates):
        """
        寻找地图宝藏网格
        """

        treasure = set()
        grid_width = frame.shape[1] // 12  # 假设图像宽度为 image_width
        grid_height = frame.shape[0] // 10  # 假设图像高度为 image_height

        for grid_number in range(1, 121):
            grid_x = (grid_number - 1) % 12 * grid_width
            grid_y = (grid_number - 1) // 12 * grid_height
            grid_x_end = grid_x + grid_width
            grid_y_end = grid_y + grid_height

            # 遍历宝藏
            for kp in keypoint_coordinates:
                # 判断是否有宝藏在当前地图网格中
                point_x, point_y = kp[0], kp[1]
                is_in_grid = (point_x >= grid_x and point_x < grid_x_end) and (point_y >= grid_y and point_y < grid_y_end)
                if is_in_grid:
                    treasure.add(grid_number)
        # 对集合进行升序排序
        sorted_list = sorted(treasure)
        return sorted_list

    def queryMapGridTransformId(self, grid_list):
        """
        地图网格转换 ID 查询
        12*10 变换 10*10
        """

        treasure = set()
        for grid in grid_list:
            row = (grid - 1) // 12
            col = (grid - 1) % 12

            if col == 0 or col == 11:  # 排除最左侧和最右侧的列
                return None

            new_grid_id = row * 10 + col
            treasure.add(new_grid_id)

        # 对集合进行升序排序
        sorted_list = sorted(treasure)
        return sorted_list

    # 图像处理
    def handler_frame(self, frame):
        while True:

            global _MAP_CLOCK # 宝藏地图锁
            global _SEND_DATA_CLOCK # 数据发送锁
            global _FRAME # 全局画幅

            # 获取图像
            data = frame.get()
            image = data["image"]

            # 缩放画幅
            image = cv2.resize(image, (1024, 768))

            # 复制出原始处理帧
            origin_frame = image.copy()

            # 更新画幅信息
            if self.FrameInfo != image.shape:
                self.FrameInfo = image.shape
                self.FrameCenter = (int(image.shape[1]) // 2, int(image.shape[0]) // 2)
                print("更新画幅尺寸及通道: ", self.FrameInfo)
                print("更新画幅中心点: ", self.FrameCenter)

            # 处理图像识别
            # 创建定位点集合
            AnchorPointSet = set()

            # 提取画幅中的所有轮廓
            contours, _ = self.detectedContours(origin_frame.copy())
            if len(contours) > 0:
                # 遍历轮廓
                for c in contours:

                    # 计算轮廓周长
                    epsilon = 0.02 * cv2.arcLength(c, True)

                    # 过滤周长
                    if int(epsilon) < 2 or int(epsilon) > 20:
                        continue

                    # 多边形逼近
                    approx = cv2.approxPolyDP(c, epsilon, True)

                    # 过滤面积噪点
                    if cv2.contourArea(approx) < 400 or cv2.contourArea(approx) > 1500:
                        continue


                    # 计算中心坐标
                    x, y, w, h = cv2.boundingRect(approx)

                    # 计算中心点
                    center = self.calcFrameCenter(x, y, w, h)

                    # 过滤矩形
                    if len(approx) != 4:
                        continue
                    else:
                        cv2.drawContours(image, [approx], -1, (0, 255, 0), 1)
                        cv2.putText(image, "c:{data1}, s:{data2}".format(data1=int(epsilon), data2=cv2.contourArea(approx)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

                    # 裁剪矩形区域
                    c_frame = image.copy()
                    c_frame = c_frame[y+2:y+h-2,x+2:x+w-2]

                    # 检查是否已存储该坐标
                    if center in AnchorPointSet:
                        continue
                    
                    # 存储坐标
                    AnchorPointSet.add(center)

            # 检查是否已存储四个定位点
            if len(AnchorPointSet) == 4 and _MAP_CLOCK is False:
                # print("识别到定位点: ", AnchorPointSet)
                # 构建逆透视矩阵
                pts_tuple = [(0, 0), (0, 0), (0, 0), (0, 0)]
                center = np.mean(list(AnchorPointSet), axis=0)
                for p in AnchorPointSet:
                    if p[0] > center[0] and p[1] > center[1]:
                        pts_tuple[3] = p
                    if p[0] < center[0] and p[1] > center[1]:
                        pts_tuple[2] = p
                    if p[0] > center[0] and p[1] < center[1]:
                        pts_tuple[1] = p
                    if p[0] < center[0] and p[1] < center[1]:
                        pts_tuple[0] = p
                input_pts = np.float32(pts_tuple)

                # 逆透视变换
                pts_frame = self.ScreenCorrection(origin_frame.copy(), input_pts)
                pts_frame = pts_frame[40:pts_frame.shape[0] - 40] # 裁剪多余区域

                 # 检查是否存在地图区域
                pts_frame = self.findMapArea(pts_frame)
                if pts_frame is None:
                    continue

                # 寻找宝藏点
                pts_frame, keypoint_coordinates = self.SpeckleFind(pts_frame)

                # 绘制宝藏轮廓定位光标
                for kc in keypoint_coordinates:
                    self.drawCross(pts_frame, kc)

                # 绘制地图网格
                self.drawGrid(pts_frame, (10, 12))

                # 在地图网格寻找宝藏所在的网格
                treasure = self.findMapTreasureGrid(pts_frame, keypoint_coordinates)

                # 空值检查
                if treasure is None:
                    continue

                print("原始 12*10 宝藏格子: ", treasure)

                # 网格 ID 变化
                treasure = self.queryMapGridTransformId(treasure)

                # 空值检查
                if treasure is None:
                    continue

                print("转换后 10*10 宝藏格子: ", treasure)
                
                # 更新宝藏到地图矩阵中
                router_dict = set()
                for i in treasure:
                    r, c = self.queryPos(i)
                    self.t_maze[r][c] = 5 # 更新宝藏
                    router_dict.add((r, c))

                print("宝藏坐标: ", router_dict)

                print("=====================================================================")


                # 构建传递数据
                temp_router = "|"
                sorted_data = sorted(router_dict, key=lambda x: x[1])
                for x, y in sorted_data:
                    temp_router = temp_router + str(x) +"-"+ str(y) + "|"

                self.movebase_router = temp_router + "1-20|"

                print("待传送宝藏坐标: ", temp_router)
    
                # 更新矫正画幅
                self.ptsFrame = pts_frame

                # 更新寻宝状态
                _MAP_CLOCK = True
                print("已锁定")

            # 创建背景
            background = image

            # 判断是否已有矫正图像
            if self.ptsFrame is not None:

                # 获取背景画面的高度和宽度
                height, width, _ = self.FrameInfo

                # 将 PTS 矫正画面缩放为原画面的 1/4 大小
                pts_resized = cv2.resize(self.ptsFrame, (width // 4, height // 4))

                # 在背景的右下角放置第二个摄像头画面
                x_offset = width - pts_resized.shape[1]
                y_offset = height - pts_resized.shape[0]
                background[y_offset:height, x_offset:width] = pts_resized

            # 显示处理后的画面
            cv2.putText(image, "fps:{fps} IS_CLOCK: {clock} Time: {time}".format(fps=data["fps"], clock=_MAP_CLOCK, time=str(datetime.datetime.now())), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
            # cv2.namedWindow("camera", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("camera", image)
            # 更新到全局
            _FRAME = image

            # 按键监听
            key_code = cv2.waitKey(1)

            # 检测是否退出
            if key_code & key_code == ord('q'):
                break

            # 检查是否保存
            if key_code & key_code == ord('n'):
                _MAP_CLOCK = False
                print("已解锁")

            global _BASE_ROS

            # 检查是否发送
            if key_code & key_code == ord('s'):
                if self.movebase_router is not None:
                    # 创建 ROS Socket 客户端
                    rosSocket = RosSocket(_BASE_ROS, 8888)
                    rosSocket.send(self.movebase_router)
                    rosSocket.close()

            if _SEND_DATA_CLOCK is True:
                _SEND_DATA_CLOCK = False
                rosSocket = RosSocket(_BASE_ROS, 8888)
                rosSocket.send(self.movebase_router)
                rosSocket.close()

            frame_queue.task_done()


class Window:
    """
    操作显示窗口
    """

    def __init__(self):
        pass

        # 创建主窗口
        self.window = tk.Tk()
        self.window.geometry("1100x900")  # 设置窗口大小

        # 创建图像显示Label
        self.label = tk.Label(self.window)
        self.label.grid(row=1, column=0, columnspan=999)

        # 创建按钮
        tk.Button(self.window, text="识别藏宝图", command=self.button_click1).grid(row=0, column=0, padx=10, pady=10)
        tk.Button(self.window, text="传送宝藏数据", command=self.button_click2).grid(row=0, column=1, padx=10, pady=10)
        
        # 启动显示
        self.show_frame()

        # 进入主事件循环
        self.window.mainloop()

    # 创建按钮点击事件的处理函数
    def button_click1(self):
        global _MAP_CLOCK
        _MAP_CLOCK = False
        print("解锁宝藏识别")

    # 创建按钮点击事件的处理函数
    def button_click2(self):
        global _SEND_DATA_CLOCK
        _SEND_DATA_CLOCK = True
        print("发送数据到小车")

    def show_frame(self):
        global _FRAME
        try:
            if _FRAME is not None:
                frame = cv2.cvtColor(_FRAME, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                img = ImageTk.PhotoImage(img)
                self.label.configure(image=img)
                self.label.image = img
            self.window.after(1, self.show_frame)

        except:
            sys.exit(0)


if __name__ == "__main__":
    # 启动 获取摄像头画面的 线程
    frame_queue = Queue()
    # cam = Camera(frame_queue)
    cam = Camera(frame_queue, 0)
    cam.run()

    # 启动处理（显示）摄像头画面的线程
    thread_show = Thread(target=ScreenProcessing, args=(frame_queue,))
    thread_show.start()

    # 启动操作显示窗口
    # thread_window = Thread(target=Window, args=())
    # thread_window.start()

    # 等待线程结束
    thread_show.join()
    # thread_window.join()
    cam.stop()
    print("已停止运行")
    sys.exit(0)

