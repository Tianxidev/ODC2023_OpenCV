#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
@File    :   hsv_debug.py
@Time    :   2023-07-23 22:49:24
@Author  :   ZhangBo<seczhangbo@163.com>
@Version :   0.1
@Contact :   HSV 调试工具 
'''
import cv2
import numpy as np
 
"""
HSV 调试工具
"""

# BGR 模式载入
image = cv2.imread('./task3.jpg') 
cv2.imshow("BGR", image)
 
hsv_low = np.array([0, 0, 0])
hsv_high = np.array([0, 0, 0])
 
def h_low(value):
    hsv_low[0] = value
 
def h_high(value):
    hsv_high[0] = value
 
def s_low(value):
    hsv_low[1] = value
 
def s_high(value):
    hsv_high[1] = value
 
def v_low(value):
    hsv_low[2] = value
 
def v_high(value):
    hsv_high[2] = value
 
cv2.namedWindow('image',cv2.WINDOW_AUTOSIZE)

cv2.createTrackbar('H low', 'image', 0, 179, h_low) 
cv2.createTrackbar('H high', 'image', 0, 179, h_high)
cv2.createTrackbar('S low', 'image', 0, 255, s_low)
cv2.createTrackbar('S high', 'image', 0, 255, s_high)
cv2.createTrackbar('V low', 'image', 0, 255, v_low)
cv2.createTrackbar('V high', 'image', 0, 255, v_high)
 
while True:
    dst = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    dst = cv2.inRange(dst, hsv_low, hsv_high)
    cv2.imshow('dst', dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()