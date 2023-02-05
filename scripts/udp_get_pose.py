import socket


# 主机ip地址
master_ip_address = '192.168.101.3'
# 主机接收数据的端口号
master_receive_port = 4514

if __name__ == "__main__":

    #创建一个udp套件字
    udp_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    #绑定本地相关信息，如果不绑定，则系统会随机分配，必须绑定本电脑的ip和port
    # local_addr =('192.168.17.128',7766)  #元组的第一个参数为本机IP，可以为空字符串，会自动生成本机IP
    local_addr =(master_ip_address, master_receive_port)
    udp_socket.bind(local_addr)
    

    # 设置循环频率
    # rate = rospy.Rate(20)
    while True:

        #等待接收方发送数据
        #rs中存储的是一个元组（接收到的数据，（发送方的ip，port））
        # print("round begin")
        rs_data = udp_socket.recvfrom(1024)
        rs_msg = rs_data[0]
        rs_addr = rs_data[1]
        # print(rs_data)
        #接收到的数据解码展示
        print(rs_msg.decode('utf-8'))
        rs_msg = rs_msg.decode('utf-8')
        # print(rs_addr)
        path_vector = rs_msg.split("|")
        print(path_vector)

        if(len(path_vector) == 4):
            stamp = float(path_vector[0])
            x = float(path_vector[1])
            y = float(path_vector[2])
            z = float(path_vector[3])
            print(x)

        # rate.sleep()

    #关闭套件字
    udp_socket.close()