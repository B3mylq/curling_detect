# curling_detect

### 激光雷达标定与冰壶位置检测
- [x] 激光雷达位置标定
- [x] 检测冰壶位置
- [ ] 卡尔曼滤波融入冰壶轨迹检测
- [ ] 优化屎山，整合头文件统一调参位置
---
### 1、编译配置
打开终端，创建或进入ROS工作空间，执行以下命令
```bash
cd src
git clone https://github.com/B3mylq/curling_detect.git
```
打开功能包curling_detect下的CMakeLists.txt，将第38行`add_library(kalman /home/b3mylq/ROS_ws/pcl_ws/src/curling_detect/include/kf_filter/kalman.cpp)`中的路径改为现在功能包中include/kf_filter/kalman.cpp所在的路径。
重新进入终端，编译文件
```bash
cd ..
catkin_make
source devel/setup.bash
```
---
### 2、激光雷达标定
准备两个方形箱子放在场地上图中橙色方块所示位置，方框两边沿与图中红线对齐\
![](https://github.com/B3mylq/curling_detect/blob/main/img/calibrate.png)
开启辅助图像节点
```bash
rosrun curling_detect prompt_graph
```
新建一个终端进入curling_detect/config目录，运行辅助标定图像
```bash
cd 【ROS工作空间】/src/curling_detect/config
rviz -d prompt_graph.rviz
```
此时可以看到如下界面，\
![](https://github.com/B3mylq/curling_detect/blob/main/img/promtp_graph.png)
蓝色为基坐标系下冰壶场地图，绿色为点云选取范围，红色的为校准辅助线\
请保证【点云选取范围内】除了箱子和地面没有其他干扰，移动激光雷达使箱子的两边沿尽量如图中所示贴近对准辅助线，使箱子接受激光扫面的四个面形如下图黄色点云所示。\
![](https://github.com/B3mylq/curling_detect/blob/main/img/calibrate_example01.png)
目前假设激光雷达【只有yaw角转动】，建议用手机水平仪或其他方法使雷达的安装面水平

3. 新建一个终端启动校准节点自动定标
```bash
rosrun curling_detect lidar_calibration
```
终端发布齐次变换矩阵并将点云转到基坐标系下显示，如下图橙色点云所示
![](https://github.com/B3mylq/curling_detect/blob/main/img/calibrate_example02.png)\
__话题对照表__
| 名称        | rviz类型   |  作用  |  源文件  |
| :--------:   | :-----:  | :----:  | :----: |
| /ground_marker     | Marker |   显示基坐标系下的冰壶场     | prompt_graph.cpp |
| /prompt_marker     |   Marker   |   显示雷达坐标系下关注的点云范围   | prompt_graph.cpp |
| /calibrate_marker  |    Marker    |  显示标定箱辅助对位条  | prompt_graph.cpp |
| /line_cloud  |    PointCloud2    |  以不同颜色显示标定箱监测到的三条直线点  | lidar_calibration.cpp |
| /check_cloud  |    PointCloud2    |  将雷达点云投射到冰壶场基坐标系下的结果，用以验证标定效果  | lidar_calibration.cpp |
---
### 3、冰壶检测
1. 新建终端，启动【禾赛64线】激光雷达节点
```bash
roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar64" frame_id:="Pandar64"
```
2. 新建终端，进入curling_detect/config目录，启动rviz可视化节点
```bash
cd 【ROS工作空间】/src/curling_detect/config
rviz -d curling_pandar64.rviz
```
左侧模块栏所有选项全部勾选，此时能看到激光雷达发布的点云数据

3. rviz中红、绿、蓝分别是x, y, z轴，按照看到的冰壶的位置，进入curling_detect/src/curling_detect.cpp中curling_init()函数，在如下两行中输入冰壶的大致位置
```cpp
curlingPose.pose.position.x = 【冰壶的大致x坐标】;
curlingPose.pose.position.y = 【冰壶的大致y坐标】;
```
新建终端，进入工作空间，编译后运行
```bash
cd 【ROS工作空间】
catkin_make
source devel/setup.bash
rosrun curling_detect curling_detect
```
此时能看到冰壶上出现红色odometry箭头，表示检测到冰壶位置