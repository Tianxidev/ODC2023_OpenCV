# -*- coding: utf-8 -*-

'''
@File    :   odc_ros.py
@Time    :   2023-08-07 9:10:23
@Author  :   ZhangBo<seczhangbo@163.com>
@Version :   0.1
@Contact :   光电宝藏识别 ROS 端
'''

import time
import cv2
import sys
import heapq
import socket
import numpy as np
from queue import Queue
from threading import Thread
import datetime
import RPi.GPIO as GPIO
from jetcam.usb_camera import USBCamera
from jetcam.csi_camera import CSICamera

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


class Camera:
    """
    摄像机读取类
    """

    def __init__(self, frame_queue, device=None):

        # 更新摄像头ID
        self.device_id = device

        # 配置摄像头参数
        if device is None:
            # self.cam = USBCamera(capture_device=0, width=640, height=480)
            self.cam = CSICamera(width=640, height=480, capture_width=640, capture_height=480, capture_fps=30)
        else:
            self.cam = cv2.VideoCapture(device)  # 获取摄像头
        
        self.frame_queue = frame_queue  # 帧队列
        self.is_running = False  # 状态标签
        self.fps = 0.0  # 实时帧率
        self._t_last = time.time() * 1000
        self._data = {}

    def capture_queue(self):

        # 捕获图像
        self._t_last = time.time() * 1000
        while self.is_running and self.cam is not None:
            if self.device_id is None:
                frame = self.cam.read()
                # ret, frame = self.cam.read()
            else:
                ret, frame = self.cam.read()
                if ret is None:
                    continue

            if self.frame_queue.qsize() < 1: 
                # 当队列中的图像都被消耗完后，再压入新的图像              
                t  = time.time() * 1000
                t_span = t - self._t_last                
                self.fps = int(1000.0 / t_span)
                self._data["image"] = frame.copy()
                self._data["fps"] = self.fps
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
        self.ThAck = False # 识别标志 False: 真 True: 假

        # 定义队伍信息
        self.ranks = "red" # 队伍类型, red: 红队 blue: 蓝队 默认: 红队

        # 红色方宝藏
        self.red_l = np.array([0, 132, 50])       # HSV 红色阈值下界
        self.red_h = np.array([179, 255, 255])    # HSV 红色阈值上界

        # 蓝色方宝藏
        self.blue_l = np.array([0, 87, 71])       # HSV 蓝色阈值下界
        self.blue_h = np.array([113, 255, 255])   # HSV 蓝色阈值上界

        self.yello_l = np.array([16, 184, 48])    # HSV 黄色阈值下界
        self.yello_h = np.array([24, 255, 255])   # HSV 黄色阈值上界

        self.green_l = np.array([51, 87, 34])     # HSV 绿色阈值下界
        self.green_h = np.array([99, 255, 255])   # HSV 绿色阈值上界

        # 宝藏识别状态引脚
        self.th_output_pin = 31

        try:

            # 设置使用的模式包含 GPIO.BOARD   GPIO.BCM  GPIO.CVM  GPIO.TEGRA_SOC 四种
            GPIO.setmode(GPIO.BOARD)
            
            # 设置pin的输入输出模式并初始化
            GPIO.setup(self.th_output_pin, GPIO.OUT, initial=GPIO.LOW)

        except Exception as e:
            print(e)
            
        # 获取 CV2 版本
        (self.major_ver, self.minor_ver, self.subminor_ver) = (cv2.__version__).split('.')

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

    # 图像处理
    def handler_frame(self, frame):
        while True:
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

            # 色彩空间切割宝藏轮廓
            if self.ranks == "red":
                conts = self.checkFrameInHSV(image, self.red_l, self.red_h)
            if self.ranks == "blue":
                conts = self.checkFrameInHSV(image, self.blue_l, self.blue_h)

            # 内部边数量
            n_len = 0

            # 宝藏真假状态
            tAck = False

            # 遍历宝藏轮廓
            for c in conts:

                # 计算轮廓周长
                epsilon = 0.1 * cv2.arcLength(c, True)

                # 过滤周长
                if int(epsilon) < 10:
                    continue

                # 拟合矩形
                approx = cv2.approxPolyDP(c, epsilon, True)

                # 计算面积
                s = cv2.contourArea(approx)
                
                # 过滤面积
                if s < 900:
                    continue

                # 计算中心坐标
                x, y, w, h = cv2.boundingRect(approx)

                # 过滤四边形以外的图形
                if len(approx) != 4:
                    continue

                # 截取宝藏区域
                tAreaFrame = image.copy()
                tAreaFrame = tAreaFrame[y+2:y+h-2,x+2:x+w-2]

                # 红队真宝藏判断
                if self.ranks == "red":
                    # 切割宝藏内的颜色 红队内部为绿色三角形为真
                    conts_g = self.checkFrameInHSV(tAreaFrame, self.green_l, self.green_h)
                    if len(conts_g) > 0:
                        tAck = True


                # 蓝队真宝藏判断
                if self.ranks == "blue":
                    # 切割宝藏内的颜色 蓝队内部为黄色圆形为真
                    conts_y = self.checkFrameInHSV(tAreaFrame, self.yello_l, self.yello_h)
                    if len(conts_y) > 0:
                        tAck = True
                                

                # 计算中心点
                center = self.calcFrameCenter(x, y, w, h)

                # 写入轮廓信息
                cv2.putText(image, "n:{n} a:{data0} c:{data1}, s:{data2}".format(n=n_len, data0=tAck, data1=int(epsilon), data2=cv2.contourArea(approx)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

                # 检查宝藏真假
                try:
                    if tAck is True:
                        self.ThAck = True
                        GPIO.output(self.th_output_pin, GPIO.HIGH)
                        print(str(datetime.datetime.now()) + "->队伍:", self.ranks, "帧率:", data["fps"]," 状态: 识别到真宝藏")
                    else:
                        self.ThAck = False
                        GPIO.output(self.th_output_pin, GPIO.LOW)
                except Exception as e:
                    print(e)

                # 绘制轮廓
                cv2.drawContours(image, [approx], -1, (0, 255, 0), 1)
                cv2.drawContours(image, c, -1, (255, 0, 0), 1)

                # 绘制十字光标
                self.drawCross(image, center)

            # 创建背景
            background = image

            # 显示处理后的画面
            cv2.putText(background, "fps:{fps} rank:{rank} TH:{TH} time:{time}".format(fps=data["fps"], TH=self.ThAck, rank=self.ranks, time=str(datetime.datetime.now())), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255))
            # cv2.namedWindow("camera", cv2.WINDOW_AUTOSIZE)    
            # cv2.imshow("camera", background)

            # 按键监听
            key_code = cv2.waitKey(1)

            # 检测是否按 ESC 退出
            if key_code & key_code == 27:
                break
            
            # 检查是否切换队伍标识
            if key_code & key_code == ord("r"):
                self.ranks = "red" # 红队
            if key_code & key_code == ord("b"):
                self.ranks = "blue" # 蓝队

            # 检查是否按下保存按钮
            if key_code & key_code == ord("s"):
                file_name = "./origin_" + str(datetime.datetime.now())+".jpg"
                cv2.imwrite(file_name, origin_frame)

            frame_queue.task_done()

if __name__ == "__main__":
    # 启动 获取摄像头画面的 线程
    frame_queue = Queue()
    cam = Camera(frame_queue)
    # cam = Camera(frame_queue, 0)
    cam.run()

    # 启动处理（显示）摄像头画面的线程
    thread_show = Thread(target=ScreenProcessing, args=(frame_queue,))
    thread_show.start()

    # 等待线程结束
    thread_show.join()
    cam.stop()
    print("已停止运行")
    sys.exit(0)

