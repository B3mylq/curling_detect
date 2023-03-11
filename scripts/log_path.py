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
import numpy as np

name = "2023-02-13-11-17-21"
current_directory = os.path.dirname(os.path.abspath(__file__))
file_path = "../records/logs/"
file_path = os.path.join(current_directory, file_path)
if not os.path.exists(file_path):
    os.mkdir(file_path)
    print("the log directory does not exist!")
file_name = file_path + name + ".txt"
if os.path.exists(file_name):
    print("the log file has already been created!")
with open(file_name, 'a') as f:
    pass

def logPath(curling_path):
    current_pose_stamped = []
    if(len(curling_path.poses) < 1):
        print("warning: current curling path is empty")
    else:
        last_timestep = curling_path.poses[-1].header.stamp.to_sec()
        current_pose_stamped.append(str(last_timestep))
        current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.x,4)))
        current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.y,4)))
        current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.z,4)))
        current_pose_stamped = ", ".join(current_pose_stamped) + "\n"
        with open(file_name, 'a') as f:
            f.write(current_pose_stamped)


if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("log_curling_path")

    master_ip_address = rospy.get_param("master_ip_address","192.168.101.3")
    node_ip_address = rospy.get_param("node_ip_address","192.168.17.128")
    master_command_port = rospy.get_param("master_command_port",1145)
    master_receive_port = rospy.get_param("master_receive_port",4514)
    master_pose_send_command = rospy.get_param("master_pose_send_command","SR")
    master_begin_record_command = rospy.get_param("master_begin_record_command","LS")
    master_end_record_command = rospy.get_param("master_end_record_command","LE")

    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("/transformed_curling_path",Path ,logPath,queue_size=10)
    #4.处理订阅的消息(回调函数)

    #5.设置循环调用回调函数
    rospy.spin()
