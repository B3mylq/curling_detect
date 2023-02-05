#! /usr/bin/env python
#coding=utf-8
#1.导包 
import rospy
import rosbag
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
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

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("receive_command")

    pub_command = rospy.Publisher("/master_command",Int8MultiArray,queue_size=100)

    master_ip_address = rospy.get_param("master_ip_address","192.168.101.3")
    node_ip_address = rospy.get_param("node_ip_address","192.168.17.128")
    master_command_port = rospy.get_param("master_command_port",1145)
    master_receive_port = rospy.get_param("master_receive_port",4514)
    master_pose_send_command = rospy.get_param("master_pose_send_command","SR")
    master_begin_record_command = rospy.get_param("master_begin_record_command","LS")
    master_end_record_command = rospy.get_param("master_end_record_command","LE")


    #创建一个udp套件字
    udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    #绑定本地相关信息，如果不绑定，则系统会随机分配，必须绑定本电脑的ip和port
    # local_addr =('192.168.17.128',7766)  #元组的第一个参数为本机IP，可以为空字符串，会自动生成本机IP
    local_addr =(node_ip_address,master_command_port)
    udp_socket.bind(local_addr)

    response_adders=(master_ip_address,master_receive_port)

    record_start_flag = 0
    record_stop_flag = 0
    udp_send_flag = 0

    # 设置循环频率
    # rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        #等待接收方发送数据
        #rs中存储的是一个元组（接收到的数据，（发送方的ip，port））
        # print("round begin")
        rs_data = udp_socket.recvfrom(1024)
        rs_msg = rs_data[0]
        rs_addr = rs_data[1]
        # print(rs_data)
        #接收到的数据解码展示
        # rs_msg = rs_msg.decode('utf-8').encode('utf-8')
        print(rs_msg)
        if(rs_msg == master_pose_send_command):
            print(rs_msg)
            udp_socket.sendto("===lidar start sending pose msg===".encode('utf-8'),response_adders)
            udp_send_flag = 1
        if(rs_msg == master_begin_record_command):
            print(rs_msg)
            udp_socket.sendto("===lidar start recording path msg===".encode('utf-8'),response_adders)
            record_start_flag = 1
            record_stop_flag = 0
        if(rs_msg == master_end_record_command):
            print(rs_msg)
            udp_socket.sendto("===lidar stop recording path msg===".encode('utf-8'),response_adders)
            record_start_flag = 0
            record_stop_flag = 1
        
        command_arr = Int8MultiArray()
        command_arr.data.append(udp_send_flag)
        command_arr.data.append(record_start_flag)
        command_arr.data.append(record_stop_flag)
        pub_command.publish(command_arr)
        print(command_arr.data)


        # rate.sleep()

    #关闭套件字
    udp_socket.close()