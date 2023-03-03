#! /usr/bin/env python
#coding=utf-8
import rospy
import rosbag
from std_msgs.msg import Int8
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import String
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import socket
import pytz 
import time
from datetime import datetime
import os 
import threading
import numpy as np

class LidarClient:
    def __init__(self,host,port,name = "Test"):
        rospy.init_node("Lidar_Client")
        self.master_pose_send_command = rospy.get_param("master_pose_send_command","SR")
        self.master_begin_record_command = rospy.get_param("master_begin_record_command","LS")
        self.master_end_record_command = rospy.get_param("master_end_record_command","LE")

        current_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = "../records/logs/"
        file_path = os.path.join(current_directory, file_path)
        if not os.path.exists(file_path):
            os.mkdir(file_path)
        self.file_path = file_path + name + ".txt"

        self.host = host 
        self.port = port 

        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.connect((host,port))
        print(f"{host} {port} connected") 
        thread = threading.Thread(target = self.receive)
        thread.start()

        self.pose_send_command = False 
        self.start_recording = False 
        self.stop_recording = False 
        self.path_recording = False
        self.pointcloud_recording = False
        self.bag = None
        self.last_path = None 
        self.path = None
        self.last_timestep = None 

        sub_path = rospy.Subscriber("/transformed_curling_path", Path ,self.process_path,queue_size=10)
        sub_pc2 = rospy.Subscriber("/rslidar_points", PointCloud2, self.process_pc2, queue_size=20)
        rospy.spin()


    def write(self,data):
        data = data.encode("utf-8")
        self.sock.send(data)
    
    def process_path(self,curling_path):
        # print('***********')
        # print(curling_path.poses)
        if self.path_recording and self.path is not None:
            current_pose_stamped = []
            delta_t = curling_path.poses[-1].header.stamp.to_sec() - self.last_timestep if self.last_timestep is not None else 0.0
            self.last_timestep = curling_path.poses[-1].header.stamp.to_sec()
            current_pose_stamped.append(str(delta_t))
            current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.x,4)))
            current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.y,4)))
            current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.z,4)))
            current_pose_stamped = "|".join(current_pose_stamped)
            self.path.append(current_pose_stamped)

        # 获取路径的最后一个（当前时刻）位置
        # # current_pose_stamped.append(str(curling_path.poses[-1].header.stamp.to_sec()))
        # current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.x,4)))
        # current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.y,4)))
        # current_pose_stamped.append(str(np.round(curling_path.poses[-1].pose.position.z,4)))

        if self.path_recording and self.bag is not None:
            self.bag.write("/transformed_curling_path", curling_path)
    
    def process_pc2(self, point_cloud2):
        if self.pointcloud_recording and self.bag is not None:
            self.bag.write("/rslidar_points", point_cloud2)


    def write_log(self,file_path,str_results):
        current_time = time.ctime(time.time())
        results = current_time + ": " + str_results
        with open(file_path, 'a') as f:
            f.write(results)
            f.write('\n') 
        

    def _get_bag_name(self):
        tz = pytz.timezone('Asia/Shanghai') #东八区
        t = datetime.fromtimestamp(int(time.time()),
            pytz.timezone('Asia/Shanghai')).strftime('%Y-%m-%d %H:%M:%S %Z%z')
        file_name = '-'.join(t.split())
        current_directory = os.path.dirname(os.path.abspath(__file__))
        file_path = "../records/rosbags/"
        path_file_name = os.path.join(current_directory, file_path) + file_name + ".bag"
        return path_file_name 

    def receive(self):
        while True:
            try:
                rs_msg = self.sock.recv(1024).decode('utf-8')
                print(rs_msg)
                if rs_msg == self.master_pose_send_command and (not self.pose_send_command):
                    self.pose_send_command = True
                    if self.last_path is not None:
                        path = ";".join(self.last_path)
                        self.write(path)
                        self.write_log(self.file_path,path)
                    self.pose_send_command = False 


                elif rs_msg == self.master_begin_record_command:
                    if self.start_recording == True:
                        self.write("WARNING! Last recording is not over yet, please end the recording before start a new record!")
                    else:
                        self.start_recording = True 
                        self.write("Start Recording")
                        self.write_log(self.file_path,'Start Recording')
                        self.path = []

                        self.path_recording = True 
                        self.pointcloud_recording = True
                        self.bag = rosbag.Bag(self._get_bag_name(),'w')

                        # self.start_recording = False

                elif rs_msg == self.master_end_record_command and (not self.stop_recording):
                    self.stop_recording = True 
                    self.start_recording = False
                    self.write("Stop Recording")
                    self.write_log(self.file_path,'Stop Recording')
                    self.last_path = self.path 
                    self.path = None 
                    self.last_timestep = None 
                    self.path_recording = False
                    self.pointcloud_recording = False
                    if self.bag is not None:
                        time.sleep(1)
                        self.bag.close()
                        self.bag = None 
                    
                    
                    self.stop_recording = False 

            except ConnectionAbortedError:
                self.sock.close()
                break
            except:
                print("ERROR")
                self.sock.close()
                break 
        self.sock.close()

    
if __name__ == '__main__':
    master_ip_address = rospy.get_param("master_ip_address","192.168.101.3")
    master_command_port = rospy.get_param("master_command_port",1145)

    client = LidarClient(host = master_ip_address , port = master_command_port)
