#! /usr/bin/env python
#coding=utf-8
#1.导包 
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
import socket

def sendPath(curling_path):
    pose_stamped = []
    pose_stamped.append(str(curling_path.poses[-1].header.stamp.to_sec()))
    pose_stamped.append(str(curling_path.poses[-1].pose.position.x))
    pose_stamped.append(str(curling_path.poses[-1].pose.position.y))
    pose_stamped.append(str(curling_path.poses[-1].pose.position.z))
    last_pose = "|".join(pose_stamped)
    print(last_pose)

    #目标IP 和端口，元组类型
    ip_adders=('192.168.1.96',7766)
    # ip_adders=('10.182.249.139',7766)
    
    udp_socket.sendto(last_pose.encode('utf-8'),ip_adders)


if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("send_curling_path")
    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("/curling_path",Path ,sendPath,queue_size=10)
    #4.处理订阅的消息(回调函数)

    #创建一个udp套件字
    udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    #5.设置循环调用回调函数
    rospy.spin()

    #关闭套件字
    udp_socket.close()