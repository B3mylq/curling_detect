#! /usr/bin/env python
#coding=utf-8
#1.导包 
import rospy
import socket
from sensor_msgs.msg import PointCloud2

def getID(point_cloud):
    frame_id = point_cloud.header.frame_id
    print(frame_id)


if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("get_frame_id")
    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("/hesai/pandar",PointCloud2 ,getID,queue_size=10)
    #4.处理订阅的消息(回调函数)

    #5.设置循环调用回调函数
    rospy.spin()
