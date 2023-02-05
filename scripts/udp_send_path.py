#! /usr/bin/env python
#coding=utf-8
#1.导包 
import rospy
import rosbag
import os
import sys
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import String
from nav_msgs.msg import Path
import socket
import time
from datetime import datetime
import pytz

# # 主机ip地址
# master_ip_address = '192.168.101.3'
# # 主机发送指令的端口号
# master_command_port = 1145
# # 主机接收数据的端口号
# master_receive_port = 4514
# # 主机表达“开始录制”的命令
# master_begin_command = "开始录制"
# # 主机表达“结束录制”的命令
# master_end_command = "结束录制"

record_start_flag = 0
record_stop_flag = 0
udp_send_flag = 0
command_arr = [0, 0, 0]
bag_created = False
pathBag = 0

def setCommand(command):
    global command_arr, udp_send_flag, record_stop_flag, record_start_flag
    command_arr = command.data
    udp_send_flag = command_arr[0]
    record_start_flag = command_arr[1]
    record_stop_flag = command_arr[2]

def sendPath(curling_path):
    global udp_send_flag, record_stop_flag, record_start_flag, bag_created
    global pathBag
    global master_ip_address, master_receive_port
 
    path_list = []
    # if(len(curling_path.poses) > 0):
    #     pose_stamped.append(str(curling_path.poses[-1].header.stamp.to_sec()))
    #     pose_stamped.append(str(curling_path.poses[-1].pose.position.x))
    #     pose_stamped.append(str(curling_path.poses[-1].pose.position.y))
    #     pose_stamped.append(str(curling_path.poses[-1].pose.position.z))
    for i in range(len(curling_path.poses)):
        pose_stamped = []
        pose_stamped.append(str(curling_path.poses[i].header.stamp.to_sec()))
        pose_stamped.append(str(curling_path.poses[i].pose.position.x))
        pose_stamped.append(str(curling_path.poses[i].pose.position.y))
        pose_stamped.append(str(curling_path.poses[i].pose.position.z))
        last_pose = "|".join(pose_stamped)
        path_list.append(last_pose)
    path = "-".join(path_list)

    #目标IP 和端口，元组类型
    ip_adders=(master_ip_address,master_receive_port)
    # ip_adders=('10.182.249.139',7766)
    
    if udp_send_flag == 1:
        udp_socket.sendto(path.encode('utf-8'),ip_adders)
        print(last_pose)
    if record_start_flag == 1:
        tz = pytz.timezone('Asia/Shanghai') #东八区
        t = datetime.fromtimestamp(int(time.time()),
            pytz.timezone('Asia/Shanghai')).strftime('%Y-%m-%d %H:%M:%S %Z%z')
        file_name = '-'.join(t.split())
        current_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = "../records/rosbags/"
        path_file_name = os.path.join(current_directory, file_path) + file_name + ".bag"

        if(bag_created == False):
            print("bag recording as: ", path_file_name)
            pathBag = rosbag.Bag(path_file_name,'w')
            bag_created = True
        
        pathBag.write("/transformed_curling_path", curling_path)
    if record_stop_flag == 1:
        print("recording stopped")
        pathBag.close()
        bag_created = False
        record_start_flag = 0
        record_stop_flag = 0


if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("send_curling_path")

    master_ip_address = rospy.get_param("master_ip_address","192.168.101.3")
    node_ip_address = rospy.get_param("node_ip_address","192.168.17.128")
    master_command_port = rospy.get_param("master_command_port",1145)
    master_receive_port = rospy.get_param("master_receive_port",4514)
    master_pose_send_command = rospy.get_param("master_pose_send_command","SR")
    master_begin_record_command = rospy.get_param("master_begin_record_command","LS")
    master_end_record_command = rospy.get_param("master_end_record_command","LE")

    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("/transformed_curling_path",Path ,sendPath,queue_size=10)
    sub_command = rospy.Subscriber("/master_command",Int8MultiArray ,setCommand,queue_size=10)
    #4.处理订阅的消息(回调函数)

    #创建一个udp套件字
    udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    #5.设置循环调用回调函数
    rospy.spin()

    #关闭套件字
    udp_socket.close()