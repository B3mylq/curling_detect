#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/visualization/pcl_visualizer.h>

#include<algorithm>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include<pcl_ros/filters/filter.h>
#include<pcl_ros/point_cloud.h>
using namespace std;

sensor_msgs::PointCloud2 point_in;
sensor_msgs::PointCloud2 pub_pc;
ros::Publisher pub_filtered_cloud;
 
void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{
	if (cloud->size() > 0)
	{
		//------------------------------------------PCL分割框架--------------------------------------------------------   
		//创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// 创建分割对象
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// 可选择配置，设置模型系数需要优化
		seg.setOptimizeCoefficients(true);
		// 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
		seg.setModelType(pcl::SACMODEL_PLANE);//设置模型类型
		seg.setMethodType(pcl::SAC_RANSAC);//设置随机采样一致性方法类型
		seg.setMaxIterations(500);//最大迭代次数
		seg.setDistanceThreshold(0.08);//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
		seg.setInputCloud(cloud);//输入所需要分割的点云对象
		//引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
		seg.segment(*inliers, *coefficients);
		//---------------------------------------------------------------------------------------------------------------
		if (inliers->indices.size() == 0)
		{
			cout << "error! Could not found any inliers!" << endl;
		}
		// extract ground
		// 从点云中抽取分割的处在平面上的点集
		pcl::ExtractIndices<pcl::PointXYZ> extractor;//点提取对象
		extractor.setInputCloud(cloud);
		extractor.setIndices(inliers);
		extractor.setNegative(true);
		extractor.filter(*cloud_filtered);
		// vise-versa, remove the ground not just extract the ground
		// just setNegative to be true
		cout << "filter done." << endl;
	}
	else
	{
		cout << "no data!" << endl;
	}
}

void seg_plane(const sensor_msgs::PointCloud2::ConstPtr& point_msg){
	point_in.header = point_msg->header;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
 
	pcl::fromROSMsg(*point_msg, *cloud);
	detectObjectsOnCloud(cloud, cloud_filtered);//执行上面定义的分割函数

	pcl::toROSMsg(*cloud_filtered, pub_pc);
    pub_pc.header = point_in.header;
    pub_filtered_cloud.publish(pub_pc); 

}
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "seg_plane");
    ros::NodeHandle nh;

	pub_filtered_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/filtered_cloud", 100, true);
    ros::Subscriber pointCLoudSub = nh.subscribe<sensor_msgs::PointCloud2>("/hesai/pandar", 100, seg_plane);

    ros::spin();


	return (0);
}