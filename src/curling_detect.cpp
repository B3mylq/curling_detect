#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>

#include <iostream>
#include <ctime>
#include "unistd.h"
#include <dlfcn.h>
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "rosbag/bag.h"
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/point_cloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
geometry_msgs::PoseStamped curlingPose;
nav_msgs::Path curlingPath, transformedCurlingPath;
float x_max, x_min, y_max, y_min, z_max, z_min, r_max, r_min;

ros::Publisher pub_filtered_cloud;
ros::Publisher pub_clustered_cloud;
ros::Publisher pub_curling_pose;
ros::Publisher pub_curling_path;
ros::Publisher pub_transformed_curling_path;
sensor_msgs::PointCloud2 point_in;
sensor_msgs::PointCloud2 pub_pc;

rosbag::Bag pathBag;
string pathbag_filename;

// 激光雷达参数(velodyne-16)
//  string lidar_frame_id = "velodyne", lidar_pc_topic = "/velodyne_points";
//  const int vertical_num = 16, horizon_num = 1800;
//  const float vertical_accuracy = 2, horizon_accuracy = 0.2;
//  const float min_angle = -15, max_angle = 15;
//  float get_vertical_accuracy(int row_id){
//      return 2;
//  }
// 激光雷达参数(rslidar-32线)
string lidar_frame_id = "rslidar", lidar_pc_topic = "/rslidar_points";
const int vertical_num = 32, horizon_num = 1800;
const float vertical_accuracy = 1, horizon_accuracy = 0.2;
const float min_angle = -16, max_angle = 15;
float get_vertical_accuracy(int row_id)
{
    return 1;
}
// 激光雷达参数(hesai-64)
// string lidar_frame_id = "Pandar64", lidar_pc_topic = "/hesai/pandar";
// const int vertical_num = 64, horizon_num = 1800;
// const float vertical_accuracy = 0.167, horizon_accuracy = 0.2;
// const float min_angle = -25, max_angle = 15;
// float get_vertical_accuracy(int row_idx)
// {
//     if (row_idx >= 5 && row_idx <= 6)
//     {
//         return 1;
//     }
//     if (row_idx >= 54 && row_idx <= 62)
//     {
//         return 1;
//     }
//     if (row_idx >= 7 && row_idx <= 53)
//     {
//         return 0.167;
//     }
//     ROS_WARN("there is no available vertical accuracy");
// }

const float ANG = 57.2957795; // 弧度制转角度的比例因数
// 冰壶参数
const double curling_vertical_size = 0.14, curling_horizon_size = 0.32;
// pcl点云对象
pcl::PCDWriter writer;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
// 位置滤波对象
pcl::PointIndices::Ptr inliers_pose(new pcl::PointIndices());
pcl::ExtractIndices<pcl::PointXYZ> extract_pose;
// 平面模型的分割对象
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
// 聚类参数
int min_size, max_size;
double cluster_tolerance;
// 为提取目的应用的搜索方法创建KdTree对象
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
/*
 * 创建一个PointIndices向量，在一个向量中包含了实际的索引信息。
 * 每个检测到的聚类的索引都保存在这里—请注意，cluster_indices是一个容器，其中包含每个检测到的集群的一个PointIndices实例。
 * 例如cluster_indices[0]包含点云中第一个群集的所有索引。
 */
std::vector<pcl::PointIndices> cluster_indices;
// 创建欧几里得集群提取器对象。
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
// 聚类结果的储存容器
vector<pcl::PointXYZ> central_poses;

char *space_to_symbol(char *str)
{
    // 处理字符串中的空格
    char *start = str;
    int space_count = 0;
    int old_length = 0;
    while (*start)
    {
        if (*start == ' ')
        {
            space_count++;
        }
        old_length++;
        start++;
    }
    int new_length = old_length;
    while (old_length >= 0)
    {
        if (str[old_length] == ' ')
        {
            str[new_length--] = '-';
        }
        else
        {
            str[new_length--] = str[old_length];
        }
        old_length--;
    }
    return str;
}

void curling_init()
{
    curlingPose.header.frame_id = lidar_frame_id;
    curlingPose.header.stamp = ros::Time::now();
    curlingPose.pose.position.x = -3.5;
    curlingPose.pose.position.y = 8;
    curlingPose.pose.position.z = 0;

    curlingPath.header = curlingPose.header;
    curlingPath.poses.clear();

    transformedCurlingPath.header = curlingPose.header;
    transformedCurlingPath.poses.clear();

    x_max = x_min = r_max = r_min = 1.2;
    y_max = y_min = 1.8;
    z_max = 0.4;
    z_min = 0.8;

    ros::param::set("/use_sim_time",true);

    // 设置平面分割的参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    // 初始化记录包裹
    char curpwd[100];
    getcwd(curpwd, 100);
    string dir_path(curpwd); // 获取当前执行程序的路径
    string file_path = "/src/curling_detect/records/rosbags/";
    time_t timep;
    time(&timep);
    char *current_time = asctime(gmtime(&timep));
    string file_name = space_to_symbol(current_time);
    file_name.pop_back();
    pathbag_filename = dir_path + file_path + file_name + ".bag";
    cout << "bag stored as: " << pathbag_filename << endl;
    // pathBag.open(pathbag_filename,rosbag::BagMode::Write);
}

void prehandler()
{
    // ROS_INFO("size before prehandler is %d", (int)cloud_filtered->size());
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);
    // ROS_INFO("size after prehandler is %d", (int)cloud_filtered->size());
    if (lidar_frame_id == "rslidar")
    {
        for (int i = 0; i < (*cloud_filtered).size(); i++)
        {
            cloud_filtered->points[i].z = 0 - cloud_filtered->points[i].z;
        }
    }
}

void position_filter()
{
    // 根据冰壶位置和运动界限（方框+球形限制）排除范围外的点

    vector<double> poseLimit = {curlingPose.pose.position.x - x_min, -(curlingPose.pose.position.x + x_max),
                                curlingPose.pose.position.y - y_min, -(curlingPose.pose.position.y + y_max),
                                curlingPose.pose.position.z - z_min, -(curlingPose.pose.position.z + z_max)};
    vector<float> limitCheck = {x_min, x_max, y_min, y_max, z_min, z_max};
    // ROS_INFO("pose limit x(%f, %f), y(%f, %f), z(%f, %f)", poseLimit[0],poseLimit[1],poseLimit[2],poseLimit[3],poseLimit[4],poseLimit[5]);

    for (int i = 0; i < (*cloud_filtered).size(); i++)
    {
        pcl::PointXYZ pt(cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].z);
        vector<double> poseComp = {pt.x, -pt.x, pt.y, -pt.y, pt.z, -pt.z};

        for (int j = 0; j < 6; j++)
        {
            if (limitCheck[j])
            {
                if (poseComp[j] < poseLimit[j])
                {
                    inliers_pose->indices.push_back(i);
                    break;
                }
            }
        }
    }

    extract_pose.setInputCloud(cloud_filtered);
    extract_pose.setIndices(inliers_pose);
    extract_pose.setNegative(true);
    // ROS_INFO("check2===============");
    extract_pose.filter(*cloud_filtered);
    // ROS_INFO("check1===============");
}

void plane_segment()
{
    //------------------------------------------PCL分割框架--------------------------------------------------------
    // 创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);    // 设置随机采样一致性方法类型
    seg.setMaxIterations(1200);            // 最大迭代次数
    seg.setDistanceThreshold(0.03);        // 设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
    seg.setInputCloud(cloud_filtered);     // 输入所需要分割的点云对象
    // 引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment(*inliers, *coefficients);
    //---------------------------------------------------------------------------------------------------------------
    if (inliers->indices.size() == 0)
    {
        cout << "error! Could not found any inliers!" << endl;
    }
    // extract ground
    // 从点云中抽取分割的处在平面上的点集
    pcl::ExtractIndices<pcl::PointXYZ> extractor; // 点提取对象
    extractor.setInputCloud(cloud_filtered);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*cloud_filtered);
    // vise-versa, remove the ground not just extract the ground
    // just setNegative to be true
    cout << "filter done." << endl;
}

void rough_cluster()
{
    double distance = sqrt(pow(curlingPose.pose.position.x, 2) + pow(curlingPose.pose.position.y, 2) + pow(curlingPose.pose.position.z, 2));
    int hori_lidar_num, ver_lidar_num;
    (curling_horizon_size / distance * ANG / horizon_accuracy > 1) ? hori_lidar_num = ceil(curling_horizon_size / distance * ANG / horizon_accuracy) : hori_lidar_num = 2;
    (curling_vertical_size / distance * ANG / vertical_accuracy > 0) ? ver_lidar_num = ceil(curling_vertical_size / distance * ANG / vertical_accuracy) : ver_lidar_num = 1;

    max_size = (hori_lidar_num + 1) * (ver_lidar_num + 1) * 1.2;
    (hori_lidar_num >= 8 && ver_lidar_num >= 2) ? min_size = ceil(hori_lidar_num * ver_lidar_num * 0.2) : min_size = 4;
    // min_size = 60;
    distance < 9.6 ? cluster_tolerance = vertical_accuracy / ANG * distance * 1.4 : cluster_tolerance = horizon_accuracy / ANG * distance * 2.7;

    tree->setInputCloud(cloud_filtered);
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    ROS_INFO("max_size: %d, min_size: %d, cluster num: %d, tolerance: %f", max_size, min_size, (int)cluster_indices.size(), cluster_tolerance);
    // ROS_INFO("size of cloud is %d", (int)cloud_filtered->size());
}

pcl::PointXYZ get_central_pose(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
{
    pcl::PointXYZ central_pose;
    central_pose.x = central_pose.y = central_pose.z = 0;

    for (int i = 0; i < point_cloud->size(); i++)
    {
        central_pose.x += point_cloud->points[i].x;
        central_pose.y += point_cloud->points[i].y;
        central_pose.z += point_cloud->points[i].z;
    }

    central_pose.x = central_pose.x / point_cloud->size();
    central_pose.y = central_pose.y / point_cloud->size();
    central_pose.z = central_pose.z / point_cloud->size();

    return central_pose;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr points_of_max_row(new pcl::PointCloud<pcl::PointXYZI>());

void extract_max_row(pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud)
{

    // 提取点云中数据量最大的一行，为作空间圆拟合作准备
    points_of_max_row->clear();
    vector<int> save_row_num(vertical_num, 0);                                 // 保存一个点云各行点的数量
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> points_of_rows(vertical_num); // 保存各行的点

    for (int row = 0; row < points_of_rows.size(); row++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr row_point_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        points_of_rows[row] = row_point_ptr;
    }

    for (int point_idx = 0; point_idx < clustered_cloud->points.size(); point_idx++)
    {

        pcl::PointXYZI pt = clustered_cloud->points[point_idx];
        double verticalAngle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
        int row_i = (vertical_num - 1) - (verticalAngle - min_angle) / 2;
        save_row_num[row_i] += 1;
        // ROS_INFO("check3===============");
        points_of_rows[row_i]->points.push_back(pt);
    }

    int maxPosition = max_element(save_row_num.begin(), save_row_num.end()) - save_row_num.begin();
    // points_of_max_row = points_of_rows[maxPosition - save_row_num.begin()];
    for (int point_idx = 0; point_idx < points_of_rows[maxPosition]->points.size(); point_idx++)
    {
        points_of_max_row->points.push_back(points_of_rows[maxPosition]->points[point_idx]);
    }
}

void cluster_extract()
{
    float intensity = 1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_all(new pcl::PointCloud<pcl::PointXYZI>);
    vector<double> max_segment;
    vector<double> circle_radius;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster_one(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto &idx : it->indices)
        {
            pcl::PointXYZI temp_point;
            temp_point.x = (*cloud_filtered)[idx].x;
            temp_point.y = (*cloud_filtered)[idx].y;
            temp_point.z = (*cloud_filtered)[idx].z;
            temp_point.intensity = intensity;
            cloud_cluster_one->push_back(temp_point);
        }

        intensity += 2;

        // 提取点云中的最大距离作为判据
        pcl::PointXYZI pmin, pmax;
        pcl::getMaxSegment(*cloud_cluster_one, pmin, pmax);
        double cloud_max_segment = pcl::euclideanDistance(pmin, pmax);
        max_segment.push_back(cloud_max_segment);
        // ROS_INFO("check2===============");
        // 提取点云最大行的拟合圆半径和圆心坐标作为判据
        // ROS_INFO("check1===============");
        extract_max_row(cloud_cluster_one);
        // ROS_INFO("check2===============");
        pcl::SampleConsensusModelCircle3D<pcl::PointXYZI>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZI>(points_of_max_row)); // 选择拟合点云与几何模型
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_circle3D);                                                                               // 创建随机采样一致性对象
        ransac.setDistanceThreshold(0.02);                                                                                                               // 设置距离阈值，与模型距离小于0.01的点作为内点
        ransac.setMaxIterations(7600);                                                                                                                   // 设置最大迭代次数
        ransac.computeModel();                                                                                                                           // 执行模型估计
        Eigen::VectorXf coefficient;
        ransac.getModelCoefficients(coefficient);
        double cloud_row_radiu = coefficient[3];
        circle_radius.push_back(cloud_row_radiu);

        pcl::PointXYZ central_pose = get_central_pose(cloud_cluster_one);
        if (cloud_max_segment > 0.12 && cloud_max_segment < 0.45 && cloud_row_radiu > 0.067 && cloud_row_radiu < 0.28)
        {
            central_poses.push_back(central_pose);
            *cloud_cluster_all = *cloud_cluster_all + *cloud_cluster_one;
            ROS_INFO("size of clustered_cloud %f is %d, x:%f, y:%f", intensity, (int)cloud_cluster_one->size(), central_pose.x, central_pose.y);
        }
        else
        {
            ROS_INFO("failed clustered_cloud %f size: %d, radiu: %f, seg: %f, x:%f, y:%f", intensity, (int)cloud_cluster_one->size(), cloud_row_radiu, cloud_max_segment, central_pose.x, central_pose.y);
        }
    }

    if (cluster_indices.size() > 0)
    {
        for (int i = 0; i < cluster_indices.size(); i++)
        {
            cout << "cluster " << to_string(i) << " max segment " << max_segment[i] << " radiu " << circle_radius[i] << "; ";
        }
        cout << endl;
    }

    cloud_cluster_all->width = cloud_cluster_all->size();
    cloud_cluster_all->height = 1;
    cloud_cluster_all->is_dense = true;
    if (cloud_cluster_all->size() > 0)
    {
        pcl::toROSMsg(*cloud_cluster_all, pub_pc);
        pub_pc.header = point_in.header;
        pub_clustered_cloud.publish(pub_pc);
    }
}

void get_curling_pose()
{
    pcl::PointXYZ last_pose;
    last_pose.x = curlingPose.pose.position.x;
    last_pose.y = curlingPose.pose.position.y;
    last_pose.z = curlingPose.pose.position.z;

    if (central_poses.size() == 0)
    {
        ROS_WARN("fail to detect curling at %f", ros::Time::now().toSec());
        pub_curling_pose.publish(curlingPose);
        return;
    }

    double min_distance = 100;
    for (int i = 0; i < central_poses.size(); i++)
    {
        double current_distance = pcl::euclideanDistance(last_pose, central_poses[i]);
        if (current_distance < min_distance)
        {
            min_distance = current_distance;
            curlingPose.header.stamp = ros::Time::now();
            curlingPose.pose.position.x = central_poses[i].x;
            curlingPose.pose.position.y = central_poses[i].y;
            curlingPose.pose.position.z = central_poses[i].z;
        }
    }

    double time_interval = ros::Time::now().toSec() - curlingPath.header.stamp.toSec();
    if ((min_distance / time_interval) > 5)
    {
        ROS_WARN("fail to track curling at %f, speed: %f", ros::Time::now().toSec(), min_distance / time_interval);
        curlingPose.header.stamp = ros::Time::now();
        curlingPose.pose.position.x = last_pose.x;
        curlingPose.pose.position.y = last_pose.y;
        curlingPose.pose.position.z = last_pose.z;
    }
    else
    {
        curlingPath.header.stamp = ros::Time::now();
        curlingPath.poses.push_back(curlingPose);
        ROS_INFO("success. ");
    }
    // ROS_INFO("min distance is %f", min_distance);
    pub_curling_pose.publish(curlingPose);
    pub_curling_path.publish(curlingPath);
}

void transform_curling_path()
{
    transformedCurlingPath = curlingPath;
    for (int i = 0; i < transformedCurlingPath.poses.size(); i++)
    {
        Eigen::Vector3d temp_pose(transformedCurlingPath.poses[i].pose.position.x,
                                  transformedCurlingPath.poses[i].pose.position.y,
                                  transformedCurlingPath.poses[i].pose.position.z);
        Eigen::Vector3d transformed_pose = transform_matrix * temp_pose;
        transformedCurlingPath.poses[i].pose.position.x = transformed_pose[0];
        transformedCurlingPath.poses[i].pose.position.y = transformed_pose[1];
        transformedCurlingPath.poses[i].pose.position.z = transformed_pose[2];
    }
    pub_transformed_curling_path.publish(transformedCurlingPath);
    // pathBag.write("/transformed_curling_path",ros::Time::now(),transformedCurlingPath);
}

void cloud_pub()
{
    pcl::toROSMsg(*cloud_filtered, pub_pc);
    pub_pc.header = point_in.header;
    pub_filtered_cloud.publish(pub_pc);
}

void reset()
{
    inliers_pose->indices.clear();
    cluster_indices.clear();

    central_poses.clear();

    cloud_filtered->clear();
}

void CurlingDetectCallback(const sensor_msgs::PointCloud2::ConstPtr &point_msg)
{

    cout << "====== round begin ======" << endl;

    pcl::fromROSMsg(*point_msg, *cloud_filtered);
    point_in.header = point_msg->header;

    prehandler();

    cloud_pub();

    position_filter();

    double distance = sqrt(pow(curlingPose.pose.position.x, 2) + pow(curlingPose.pose.position.y, 2) + pow(curlingPose.pose.position.z, 2));
    if (distance < 18)
    {
        plane_segment();
    }

    rough_cluster();

    cluster_extract();

    get_curling_pose();

    transform_curling_path();

    reset();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "curling_detection");
    ros::NodeHandle nh;
    curling_init();
    bool calibrate_flag = nh.param("lidar_calibrate_flag", false);
    if (calibrate_flag)
    {
        vector<double> temp_tf_params;
        vector<double> tf_params = nh.param("lidar_tf_params", temp_tf_params);
        if (tf_params.size() != 16)
        {
            ROS_WARN("incorrect number of tf params");
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    transform_matrix(i, j) = tf_params[i + j];
                }
            }
        }
    }
    else
    {
        ROS_WARN("no transform matrix received");
        transform_matrix = Eigen::Isometry3d::Identity();
    }

    pub_filtered_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/filtered_cloud", 100, true);
    pub_clustered_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/clustered_cloud", 100, true);
    pub_curling_pose = nh.advertise<geometry_msgs::PoseStamped>("/curling_pose", 10, true);
    pub_curling_path = nh.advertise<nav_msgs::Path>("/curling_path", 10, true);
    pub_transformed_curling_path = nh.advertise<nav_msgs::Path>("/transformed_curling_path", 10, true);
    ros::Subscriber pointCLoudSub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_pc_topic, 100, CurlingDetectCallback);

    ros::spin();

    // pathBag.close();

    return 0;
}