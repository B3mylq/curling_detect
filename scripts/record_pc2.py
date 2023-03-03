#! /usr/bin/env python
#coding=utf-8
#1.导包 
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import socket
import rosbag
import time
from datetime import datetime
import pytz
import os
import sys


pc2Bag = 0

def callback(point_cloud2):
    pathBag.write("/rslidar_points", point_cloud2)

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("receive_curling_path")

    tz = pytz.timezone('Asia/Shanghai') #东八区
    t = datetime.fromtimestamp(int(time.time()),
        pytz.timezone('Asia/Shanghai')).strftime('%Y-%m-%d %H:%M:%S %Z%z')
    file_name = '-'.join(t.split())
    current_directory = os.path.dirname(os.path.abspath(__file__))
    file_path = "../records/rosbags/"
    path_file_name = os.path.join(current_directory, file_path) + file_name + ".bag"

    print("bag recording as: ", path_file_name)
    pathBag = rosbag.Bag(path_file_name,'w')

    sub = rospy.Subscriber("/rslidar_points",
                                     PointCloud2,
                                     callback, queue_size=5)
    
    rospy.spin()

    pathBag.close()

    

    
