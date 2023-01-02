#include<iostream>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/PCLPointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/common/transforms.h>
#include<pcl_ros/filters/filter.h>
#include<pcl_ros/point_cloud.h>
#include<opencv2/core/eigen.hpp>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include"depth_make/make_pointxyzitr.h"
//make_pointxyzitr.h为自定义PCL点云消息pcl::PointCloud<pcl::PointXYZITR>的头文件，该自定义点云消息中的每个点都包括了XYZ坐标、反射强度intensity、时间戳time_stamp和点所在线束ring。
using namespace std;
ros::Publisher pub;
 
void PointCallback(const sensor_msgs::PointCloud2::ConstPtr& point_msg)
{
    // ROS_INFO("check1==================");
    //首先将接收到的ROS格式的点云消息sensor_msgs::PointCloud2转化为PCL格式的消息    PointCloud<pcl::PointXYZITR>
    pcl::PointCloud<pcl::PointXYZITR> cloud_msg;
    pcl::fromROSMsg(*point_msg, cloud_msg);
    // ROS_INFO("check2=================="); 
    cloud_msg.header.frame_id = "map";
 
    float range; //range为该点的深度值
    int row_i, col_i; 
    int scan_num = 16;
    int horizon_num = 1800;
    const float PI = 3.1415926;
    const float ANG = 57.2957795; //弧度制转角度的比例因数
    const float horizon_accuracy = 0.2;
    //根据雷达型号，创建Mat矩阵，由于在此使用的雷达为128线，每条线上有1281个点，所以创建了一个大小为128*1281尺寸的矩阵，并用0元素初始化。
    cv::Mat range_mat = cv::Mat(scan_num, horizon_num, CV_8UC3, cv::Scalar::all(0));
    
    //遍历每个点，计算该点对应的Mat矩阵中的索引，并为该索引对应的Mat矩阵元素赋值
    for(const auto pt : cloud_msg.points)
    {
        if(pt.ring != 0)  //忽略NAN点(NAN点的ring值为0)
        {
            row_i = pt.ring;
            // ROS_INFO("ring is %d", row_i);
            // col_i = ((128.1/2) - atan2(pt.y, pt.x) * ANG) / 0.1;
            col_i = ((horizon_num*horizon_accuracy/2) - atan2(pt.y, pt.x) * ANG) / horizon_accuracy;
            //计算该点的深度值
            range = (float)(sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z))/100*255; 
            //忽略索引不合法的点
            if(row_i < 0 || row_i >= scan_num)
                continue;
            if(col_i < 0 || col_i >= horizon_num)
                continue;
            range_mat.at<float>(row_i, col_i) = range;
            //如果想转化为彩色的深度图，可以注释上面这一句，改用下面这一句；
            //range_mat.at<cv::Vec3b>(row_i, col_i) = cv::Vec3b(254-int(pt.x *2), 254- int(fabs(pt.y) / 0.5), 254-int(fabs((pt.z + 1.0f) /0.05)));
        }
    }
    
    cv::namedWindow("map",cv::WINDOW_NORMAL);//AUTOSIZE //创建一个窗口，用于显示深度图
    cv::imshow("map",range_mat); //在这个窗口输出图片。
    cv::waitKey(10); //设置显示时间
 
    pub.publish(cloud_msg); //发布点云
}
 
int main (int argc, char **argv)
    {
	ros::init (argc, argv, "vlp_16");
	ros::NodeHandle nh;
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZITR>>("pcl_points", 100, true);
    ros::Subscriber mapSub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, PointCallback);
    ros::spin();
    return 0;
}