#! /usr/bin/env python
#coding=utf-8
#1.导包 
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import socket

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("receive_curling_path")
    #3.实例化 发布者 对象
    pub = rospy.Publisher("/curling_path_udp",Path,queue_size=100)
    pub_stamped = rospy.Publisher("/curling_pose_udp",PoseStamped,queue_size=100)
    curling_path = Path()
    curling_pose = PoseStamped()

    #创建一个udp套件字
    udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    #绑定本地相关信息，如果不绑定，则系统会随机分配，必须绑定本电脑的ip和port
    # local_addr =('192.168.17.128',7766)  #元组的第一个参数为本机IP，可以为空字符串，会自动生成本机IP
    local_addr =('10.182.249.139',7766)
    udp_socket.bind(local_addr)
    

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
        # print(rs_msg.decode('utf-8'))
        # print(rs_addr)
        
        # print(path_vector)
        curling_path.header.frame_id = "Pandar64"
        curling_path.header.stamp = rospy.Time.now()
        curling_path.header.seq = 50

        path_vector = rs_msg.split("-")
        for i in range(len(path_vector)):
            temp_pose = path_vector[i].split("|")
            curling_pose.header.frame_id = "Pandar64"
            curling_pose.header.stamp = rospy.Time.from_sec(float(temp_pose[0]))
            # curling_pose.header.stamp = rospy.Time.now()
            curling_pose.pose.position.x = float(temp_pose[1])
            curling_pose.pose.position.y = float(temp_pose[2])
            curling_pose.pose.position.z = float(temp_pose[3])
            curling_path.poses.append(curling_pose)

        pub.publish(curling_path)
        pub_stamped.publish(curling_pose)
        print(curling_path.poses[-1].header.stamp.to_sec())


        # rate.sleep()

    #关闭套件字
    udp_socket.close()