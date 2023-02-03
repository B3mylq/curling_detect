#! /usr/bin/env python
#coding=utf-8
#1.导包 
import rospy
from std_msgs.msg import Int8
from nav_msgs.msg import Path
import socket


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

udp_send_flag = 0

def setCommand(send_flag):
    global udp_send_flag
    udp_send_flag = send_flag.data

def sendPath(curling_path):
    global udp_send_flag, master_ip_address, master_receive_port

    pose_stamped = []
    pose_stamped.append(str(curling_path.poses[-1].header.stamp.to_sec()))
    pose_stamped.append(str(curling_path.poses[-1].pose.position.x))
    pose_stamped.append(str(curling_path.poses[-1].pose.position.y))
    pose_stamped.append(str(curling_path.poses[-1].pose.position.z))
    last_pose = "|".join(pose_stamped)

    #目标IP 和端口，元组类型
    ip_adders=(master_ip_address,master_receive_port)
    # ip_adders=('10.182.249.139',7766)
    
    if udp_send_flag == 1:
        udp_socket.sendto(last_pose.encode('utf-8'),ip_adders)
        print(last_pose)


if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("send_curling_path")

    master_ip_address = rospy.get_param("master_ip_address","192.168.101.3")
    node_ip_address = rospy.get_param("node_ip_address","192.168.17.128")
    master_command_port = rospy.get_param("master_command_port",1145)
    master_receive_port = rospy.get_param("master_receive_port",4514)
    master_begin_command = rospy.get_param("master_begin_command","start")
    master_end_command = rospy.get_param("master_end_command","stop")

    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("/curling_path",Path ,sendPath,queue_size=10)
    sub_command = rospy.Subscriber("/master_command",Int8 ,setCommand,queue_size=10)
    #4.处理订阅的消息(回调函数)

    #创建一个udp套件字
    udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    #5.设置循环调用回调函数
    rospy.spin()

    #关闭套件字
    udp_socket.close()