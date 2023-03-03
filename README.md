# curling_detect

### 激光雷达标定与冰壶位置检测
- [x] 激光雷达位置标定
- [x] 检测冰壶位置
- [ ] 卡尔曼滤波融入冰壶轨迹检测
- [ ] 优化屎山，整合头文件统一调参位置

更新日志2023/2/3：封装socket通信模块，冰壶位置的实时卡尔曼滤波由于程序时间计算上有bug未解决而暂时未更新\
更新日志2023/2/5：更新本地记录功能，详见4.4；更新socket收发轨迹的内容和格式，详见4.3\
更新日志2023/2/7：更新RS-Helios-1615
更新日志2023/3/3：根据新的标定要求更新圆柱标定物的标定方法，更新socket通讯代码的封装，优化部分其他封装
雷达参数，修复已知的因为无效点造成的bug。

---
### 1、编译配置
打开终端，创建或进入ROS工作空间，执行以下命令克隆项目并编译
```bash
cd src
git clone https://github.com/B3mylq/curling_detect.git
cd ..
catkin_make
source devel/setup.bash
```

---
### 2、激光雷达标定：确定雷达在基坐标系下的位置
1. 标定原理：通过四条线条确定两个基坐标系下已知的两个交点【即下文中的“标定点”】，通过两个交点在雷达坐标系下的位置确定雷达相对基坐标系的精确位置。
2. 参数与标定物准备：打开curling_detect/launch/lidar_calibration.launch文件，确定两个标定点在基坐标系下的准确坐标和雷达在基坐标系下的粗略预期坐标\
【注意：请将y坐标更小的标定点作为标定点1】\
【标定点的坐标需要使用者在标定前精确确定，而雷达坐标只是粗略的预期，通过标定程序来确定】
<!-- ![](https://github.com/B3mylq/curling_detect/blob/main/img/calibrate.png) -->
3. 标定物与雷达放置：准备两个方形箱子放在场地上，使朝向雷达的直角点位于场地上标定点的位置\
打开一个新终端运行辅助图像节点，将rviz左侧模块全部勾选上
```bash
roslaunch curling_detect lidar_calibration.launch
```
&emsp;&emsp;此时可以看到如下界面，\
![](https://github.com/B3mylq/curling_detect/blob/main/img/promtp_graph.png)
&emsp;&emsp;蓝色为基坐标系下冰壶场地图，绿色为点云选取范围，红色的为校准辅助线\
&emsp;&emsp;请保证【点云选取范围内】除了箱子和地面没有其他干扰，【移动激光雷达】使箱子的直角点【即标定点】与对准辅助线形成的直角交点尽量重合，箱子接受激光扫面的四个面形如下图黄色点云所示。\
![](https://github.com/B3mylq/curling_detect/blob/main/img/calibrate_example01.png)
&emsp;&emsp;目前假设激光雷达【只有yaw角转动】，建议用手机水平仪或其他方法使雷达的安装面水平

4. 新建一个终端启动校准节点自动定标
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
1. 新建终端，启动激光雷达节点【如下以禾赛64线雷达为例：】
```bash
roslaunch hesai_lidar hesai_lidar.launch lidar_type:="Pandar64" frame_id:="Pandar64"
```
2. 新建终端，进入curling_detect/config目录，按照如下对应关系启动rviz可视化节点

| 雷达        | rviz文件名称   |
| :--------:   | :-----:  |
| 禾赛64线     | curling_pandar64.rviz |
| RS-Helios-1615  |   curling_32.rviz   | 
```bash
cd 【ROS工作空间】/src/curling_detect/config
rviz -d 【rviz文件名称】
```
&emsp;&emsp;左侧模块栏所有选项全部勾选，此时能看到激光雷达发布的点云数据

3. rviz中红、绿、蓝分别是x, y, z轴，按照看到的冰壶的位置，进入curling_detect/src/curling_detect.cpp中curling_init()函数，在如下两行中输入冰壶的大致位置
```cpp
curlingPose.pose.position.x = 【冰壶的大致x坐标】;
curlingPose.pose.position.y = 【冰壶的大致y坐标】;
```
&emsp;&emsp;新建终端，进入工作空间，编译后运行
```bash
cd 【ROS工作空间】
catkin_make
source devel/setup.bash
rosrun curling_detect curling_detect
```
&emsp;&emsp;此时能看到冰壶上出现红色odometry箭头，表示检测到冰壶位置


---
### 4、socket收发
1. 配置python文件权限：终端中进入curling_detect/scripts/文件夹
```bash
cd 【ROS工作空间】/src/curling_detect/scripts
chmod +x *.py
```
2. 配置网络参数：打开curling_detect/launch/socket_control.launch文件，关注“<param” 开头的几行，根据注释在“value=”选项下配置网络参数
3. 主机解码方法：接收方的解码方法在curling_detect/scripts/udp_get_path.py文件中，打开该文件，在main函数前修改网络参数(参数含义与4.2中参数相同)，之后可以复制到主机上运行\
解码得到一个轨迹列表，其中每个元素为含有四个浮点数元素的列表，四个数字依次为时间戳和该时间上的x、y、z坐标
4. 运行socket收发模块：新建终端运行如下命令。
```bash
roslaunch curling_detect socket_control.launch
```
&emsp;&emsp;此时从机进入等待发送的模式，在收到激光雷达检测程序发送的Path后，主机发送以下命令控制从机的通讯内容。
| 命令        | 作用   |
| :--------:   | :-----:  |
| SR     | 从机回复“===lidar start sending pose msg===”，并开始向主机发送以获取的冰壶轨迹信息 |
| LS     |   从机回复“===lidar start recording path msg===”，并开始在curling_detect/records/rosbags/下记录检测到路径的rosbag，文件名为受到“LS”命令的【北京时间】。   |
| LE  |    从机回复“===lidar stop recording path msg===”，并停止记录冰壶轨迹信息  |    

&emsp;&emsp;主机的命令可以在4.2中更改，但先不要使用中文！