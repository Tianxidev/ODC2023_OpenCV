# ODC2023_OpenCV
2023光电设计大赛小车题视觉部分实现及 A* 路径规划实现

## 说明

由于我们太菜，小车底盘控制部分总有些奇奇怪怪的问题，因此只开源此视觉部分

前期我们进行过 SLAM 激光雷达建图然后进行导航，但是效果不理想因此放弃, maps 目录下即为激光雷达扫描建立的地图数据

我们采用平板（Suface）进行藏宝图的识别，然后将宝藏点传送至底盘 ROS 控制部分, 路径规划在 `odc_pc.py` 有实现，但实际使用在 ROS 底盘控制部分，这里只发送宝藏对应矩阵坐标到 ROS 订阅

## 启动藏宝图视觉识别

```
python3 odc_pc.py
```

## 启动小车实时宝藏识别

```
python3 odc_ros.py
```

## 启动 ROS SOCKET 服务器

```
roslaunch odc_ap socket_server.launch
```

## 主要目录说明

- launch: ROS launch 启动文件目录
- script: ROS script 脚本文件目录
- images: 参观赛场时拍摄的宝藏图像
- maps: 地图目录