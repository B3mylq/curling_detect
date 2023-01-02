#include<iostream>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sensor_msgs/PointCloud2.h>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/PCLPointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/common/transforms.h>
#include<pcl_ros/filters/filter.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/common/common.h>

#include<opencv2/core/eigen.hpp>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include"depth_make/make_pointxyzitr.h"
//make_pointxyzitr.h为自定义PCL点云消息pcl::PointCloud<pcl::PointXYZITR>的头文件，该自定义点云消息中的每个点都包括了XYZ坐标、反射强度intensity、时间戳time_stamp和点所在线束ring。
 
using namespace std;
typedef pcl::PointXYZ  PointType;
// ros::Publisher pub;
const int N_SCAN = 16;
const int Horizon_SCAN = 1800;
const float ang_res_x = 0.2;
const float ang_res_y = 2.0;
const float ang_bottom = 15.0+0.1;
const int groundScanInd = 7;
const float sensorMinimumRange = 1.0;

float verticalAngle, horizonAngle, range; //range为该点的深度值
//根据雷达型号，创建Mat矩阵，由于在此使用的雷达为128线，每条线上有1281个点，所以创建了一个大小为128*1281尺寸的矩阵，并用0元素初始化。
// cv::Mat range_mat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8UC3, cv::Scalar::all(0));
cv::Mat range_mat;
pcl::PointCloud<PointType>::Ptr cloud_msg;

void initialize(){
    cloud_msg.reset(new pcl::PointCloud<PointType>());
}
 
void PointCallback(const sensor_msgs::PointCloud2ConstPtr& point_msg)
{
    // ROS_INFO("check1==================");
    //首先将接收到的ROS格式的点云消息sensor_msgs::PointCloud2转化为PCL格式的消息    PointCloud<pcl::PointXYZITR>
    // pcl::PointCloud<pcl::PointXYZITR> cloud_msg;
    
    pcl::fromROSMsg(*point_msg, *cloud_msg);
    cloud_msg->header.frame_id = "map";

    size_t rowIdn, columnIdn, index, cloudSize; 
    PointType thisPoint;

    cloudSize = cloud_msg->points.size();

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = cloud_msg->points[i].x;
        thisPoint.y = cloud_msg->points[i].y;
        thisPoint.z = cloud_msg->points[i].z;
        // find the row and column index in the iamge for this point

        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < sensorMinimumRange){
            continue;
        }
        
        range_mat.at<float>(rowIdn, columnIdn) = range;
    }
    
    // cv::namedWindow("map",CV_WINDOW_NORMAL);//AUTOSIZE //创建一个窗口，用于显示深度图
    // cv::imshow("map",range_mat); //在这个窗口输出图片。
    // cv::waitKey(10); //设置显示时间
 
    // pub.publish(cloud_msg); //发布点云
    ROS_INFO("check4=================="); 
    cloud_msg->clear();
    range_mat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(0));
}
 
int main (int argc, char **argv)
{
	ros::init (argc, argv, "vlp_16");
	ros::NodeHandle nh;
    initialize();
    // pub = nh.advertise<pcl::PointCloud<pcl::PointXYZITR>>("pcl_points", 100, true);
    ros::Subscriber mapSub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, PointCallback);
    ros::spin();
    return 0;
}