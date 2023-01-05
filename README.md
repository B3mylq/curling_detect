# curling_detect

### 激光雷达标定与冰壶位置检测
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
开启辅助图像节点
```bash
rosrun curling_detect prompt_graph
```
新建一个终端进入curling_detect/config目录，运行辅助标定图像
```bash
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
终端发布齐次变换矩阵并将点云转到基坐标系下显示，如下图橙色点云所示\
![](https://github.com/B3mylq/curling_detect/blob/main/img/calibrate_example02.png)