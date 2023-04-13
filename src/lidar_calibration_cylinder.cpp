#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> // 拟合直线
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/point_cloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

// 激光雷达参数(hesai-64)
// string lidar_frame_id = "Pandar64", lidar_pc_topic = "/hesai/pandar";
// const int vertical_num = 64, horizon_num = 1800;
// const float vertical_accuracy = 0.167, horizon_accuracy = 0.2;
// const float min_angle = -25, max_angle = 15;
// 激光雷达参数(rslidar-32线)
string lidar_frame_id = "rslidar", lidar_pc_topic = "/rslidar_points";
const int vertical_num = 32, horizon_num = 1800;
const float vertical_accuracy = 1, horizon_accuracy = 0.2;
const float min_angle = -16, max_angle = 15;

const float ANG = 57.2957795; // 弧度制转角度的比例因数
double marker1_x, marker1_y, marker2_x, marker2_y, lidar_x, lidar_y, lidar_yaw;
bool in_real_machine;

pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr lane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr check_cloud(new pcl::PointCloud<pcl::PointXYZI>);
vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ransac_clouds;
ros::Publisher pub_line_cloud;
ros::Publisher pub_check_cloud;
sensor_msgs::PointCloud2 line_cloud_pub;
sensor_msgs::PointCloud2 check_cloud_pub;

Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
Eigen::Isometry3d transform_matrix_ave;
int round_count = 0;

vector<double> tf_params;
bool calibrate_flag = false;

void prehandler()
{
    // ROS_INFO("size before prehandler is %d", (int)cloud_filtered->size());
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*input_cloud, *input_cloud, mapping);
    // ROS_INFO("size after prehandler is %d", (int)cloud_filtered->size());
    if (lidar_frame_id == "rslidar")
    {
        for (int i = 0; i < (*input_cloud).size(); i++)
        {
            input_cloud->points[i].z = 0 - input_cloud->points[i].z;
            input_cloud->points[i].x = 0 - input_cloud->points[i].x;
        }
    }
}

void rotate_pose(geometry_msgs::Point &pose, Eigen::Matrix3d rotation_matrix){
    Eigen::Vector3d temp_pose(pose.x, pose.y, pose.z);
    Eigen::Vector3d transformed_pose = rotation_matrix*temp_pose;

    pose.x = transformed_pose[0];
    pose.y = transformed_pose[1];
    pose.z = transformed_pose[2];
}

pcl::PointCloud<pcl::PointXYZI>::Ptr position_filter(double x, double y, double z)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr inliers_pose(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZI> extract_pose;
    geometry_msgs::Point origin;
    double range = 1;
    origin.z = z;
    origin.y = y;
    origin.x = x;

    vector<double> poseLimit = {origin.x - range, -(origin.x + range),
                                origin.y - range, -(origin.y + range),
                                origin.z - range, -(origin.z + range)};
    vector<double> limitCheck = {range, range, range, range, 0, 0};
    // ROS_INFO("pose limit x(%f, %f), y(%f, %f), z(%f, %f)", poseLimit[0],poseLimit[1],poseLimit[2],poseLimit[3],poseLimit[4],poseLimit[5]);

    for (int i = 0; i < (*input_cloud).size(); i++)
    {
        pcl::PointXYZ pt(input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z);
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

    extract_pose.setInputCloud(input_cloud);
    extract_pose.setIndices(inliers_pose);
    extract_pose.setNegative(true);
    extract_pose.filter(*filtered_cloud);
    return filtered_cloud;
}

void vertical_angle_filter(double target_angle)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_points(new pcl::PointCloud<pcl::PointXYZI>);

    vector<vector<int>> idx_of_cols(horizon_num);
    for (int idx = 0; idx < input_cloud->points.size(); idx++)
    {
        pcl::PointXYZI pt = input_cloud->points[idx];
        int col_i = ((horizon_num * horizon_accuracy / 2) - atan2(pt.y, pt.x) * ANG) / horizon_accuracy;
        idx_of_cols[col_i].push_back(idx);
    }

    for (int col = 0; col < idx_of_cols.size(); col++)
    {
        double min_dist = 420;
        pcl::PointXYZI target_pt;

        for (int i = 0; i < idx_of_cols[col].size(); i++)
        {
            int idx = idx_of_cols[col][i];
            pcl::PointXYZI pt = input_cloud->points[idx];
            double verticalAngle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
            double angle_dist = abs(verticalAngle - target_angle);
            if (angle_dist < min_dist)
            {
                min_dist = angle_dist;
                target_pt = pt;
            }
        }

        if (min_dist < 420)
        {
            target_points->points.push_back(target_pt);
        }
    }
    input_cloud->clear();
    *input_cloud += *target_points;
}

void vertical_angle_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr input, double target_angle)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_points(new pcl::PointCloud<pcl::PointXYZI>);

    vector<vector<int>> idx_of_cols(horizon_num);
    for (int idx = 0; idx < input->points.size(); idx++)
    {
        pcl::PointXYZI pt = input->points[idx];
        int col_i = ((horizon_num * horizon_accuracy / 2) - atan2(pt.y, pt.x) * ANG) / horizon_accuracy;
        idx_of_cols[col_i].push_back(idx);
    }

    for (int col = 0; col < idx_of_cols.size(); col++)
    {
        double min_dist = 420;
        pcl::PointXYZI target_pt;

        for (int i = 0; i < idx_of_cols[col].size(); i++)
        {
            int idx = idx_of_cols[col][i];
            pcl::PointXYZI pt = input->points[idx];
            double verticalAngle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
            double angle_dist = abs(verticalAngle - target_angle);
            if (angle_dist < min_dist)
            {
                min_dist = angle_dist;
                target_pt = pt;
            }
        }

        if (min_dist < 420)
        {
            target_points->points.push_back(target_pt);
        }
    }
    input->clear();
    *input += *target_points;
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

Eigen::Vector2d get_curling_centre(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
{
    Eigen::Vector2d result(-1, -1);

    pcl::SampleConsensusModelCircle3D<pcl::PointXYZI>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZI>(point_cloud)); // 选择拟合点云与几何模型
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_circle3D);                                                                         // 创建随机采样一致性对象
    ransac.setDistanceThreshold(0.02);                                                                                                         // 设置距离阈值，与模型距离小于0.01的点作为内点
    ransac.setMaxIterations(7600);                                                                                                             // 设置最大迭代次数
    ransac.computeModel();                                                                                                                     // 执行模型估计
    Eigen::VectorXf coefficient;
    ransac.getModelCoefficients(coefficient);

    result[0] = coefficient[0];
    result[1] = coefficient[1];
    double curling_radiu = coefficient[3];

    if(0.05 < curling_radiu && curling_radiu < 0.2){
        return result;
    }else{
        pcl::PointXYZ central_pose = get_central_pose(point_cloud);
        result[0] = central_pose.x;
        result[1] = central_pose.y;
        return result;
    }
}

Eigen::Isometry3d get_rotation(Eigen::Vector3d vectorBefore, Eigen::Vector3d vectorAfter)
{
    Eigen::Matrix3d rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();

    Eigen::Isometry3d homo_tf = Eigen::Isometry3d::Identity();
    homo_tf.rotate(rotMatrix);

    return homo_tf;
}

Eigen::Isometry3d get_translation(Eigen::Isometry3d rot_tf, Eigen::Vector3d vectorBefore, Eigen::Vector3d vectorAfter)
{
    Eigen::Vector3d temp_vector = rot_tf * vectorBefore;
    Eigen::Vector3d translation(vectorAfter[0] - temp_vector[0],
                                vectorAfter[1] - temp_vector[1],
                                vectorAfter[2] - temp_vector[2]);
    Eigen::Isometry3d homo_tf = rot_tf;
    homo_tf.pretranslate(translation);

    return homo_tf;
}

void calibrate(Eigen::Vector2d curling1_centre, Eigen::Vector2d curling2_centre)
{
    Eigen::Vector3d vectorBefore(curling2_centre[0] - curling1_centre[0], 
                                 curling2_centre[1] - curling1_centre[1], 0), 
                    vectorAfter(marker2_x - marker1_x, marker2_y - marker1_y, 0);
    transform_matrix = get_rotation(vectorBefore, vectorAfter);
    vectorBefore << curling1_centre[0], curling1_centre[1], 0;
    vectorAfter << marker1_x, marker1_y, 0;
    transform_matrix = get_translation(transform_matrix, vectorBefore, vectorAfter);
    std::cout << "transform_matrix: " << std::endl
              << transform_matrix.matrix() << std::endl;

    round_count += 1;
    transform_matrix_ave.matrix() += transform_matrix.matrix();

    if(round_count > 10){
        transform_matrix_ave.matrix() = transform_matrix_ave.matrix()/round_count;
        std::cout << "transform_matrix_ave: " << std::endl
              << transform_matrix_ave.matrix() << std::endl;
        calibrate_flag = true;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr points_transform(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Isometry3d tf)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_points(new pcl::PointCloud<pcl::PointXYZI>);
    *transformed_points += *point_cloud;
    for (int i = 0; i < transformed_points->points.size(); i++)
    {
        Eigen::Vector3d temp_point(transformed_points->points[i].x, transformed_points->points[i].y, transformed_points->points[i].z);
        Eigen::Vector3d point_transformed = tf * temp_point;
        transformed_points->points[i].x = point_transformed[0];
        transformed_points->points[i].y = point_transformed[1];
        transformed_points->points[i].z = point_transformed[2];
    }
    return transformed_points;
}

void CalibrateCallback(const sensor_msgs::PointCloud2::ConstPtr &point_msg)
{
    ROS_INFO("round begin======");

    pcl::fromROSMsg(*point_msg, *input_cloud);

    prehandler();

    // vertical_angle_filter(0);

    if(in_real_machine){
        pcl::toROSMsg(*input_cloud, line_cloud_pub);
        line_cloud_pub.header.stamp = ros::Time::now();
        line_cloud_pub.header.frame_id = lidar_frame_id;
        pub_line_cloud.publish(line_cloud_pub);
    }

    int size;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    geometry_msgs::Point offset1, offset2;
    offset1.x = marker1_x - lidar_x; offset1.y = marker1_y - lidar_y;
    offset2.x = marker2_x - lidar_x, offset2.y = marker2_y - lidar_y;
    Eigen::AngleAxisd rotation_vector(-lidar_yaw/ANG, Eigen::Vector3d (0, 0, 1));
    rotate_pose(offset1, rotation_vector.matrix());
    rotate_pose(offset2, rotation_vector.matrix());
    ROS_INFO("after rotation1:(%f, %f, %f)", offset1.x, offset1.y, offset1.z);
    *filtered_cloud1 += *(position_filter(offset1.x, offset1.y, 0));
    vertical_angle_filter(filtered_cloud1, 0);
    *filtered_cloud2 += *(position_filter(offset2.x, offset2.y, 0));
    vertical_angle_filter(filtered_cloud2, 0);
    // *filtered_cloud1 += *(position_filter(3, 13, 0));
    // *filtered_cloud2 += *(position_filter(2, -9, 0));
    Eigen::Vector2d curling1_centre, curling2_centre;
    curling1_centre = get_curling_centre(filtered_cloud1);
    curling2_centre = get_curling_centre(filtered_cloud2);
    // ROS_INFO("centre pose 1 is %f, %f", curling1_centre[0], curling1_centre[1]);
    calibrate(curling1_centre, curling2_centre);

    input_cloud->clear();
    *input_cloud += *filtered_cloud1;
    *input_cloud += *filtered_cloud2;

    size = input_cloud->points.size();
    ROS_INFO("size of input_cloud is %d", size);

    // pcl::fromROSMsg(*point_msg, *input_cloud);
    check_cloud = points_transform(input_cloud, transform_matrix);
    pcl::toROSMsg(*check_cloud, check_cloud_pub);
    check_cloud_pub.header.stamp = ros::Time::now();
    check_cloud_pub.header.frame_id = lidar_frame_id;
    pub_check_cloud.publish(check_cloud_pub);

    lane_cloud->clear();
    check_cloud->clear();
    ransac_clouds.clear();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_calibrate_cylinder");
    ros::NodeHandle nh;

    marker1_x = ros::param::param("marker1_x", 8.2296);
    marker1_y = ros::param::param("marker1_y", 0);
    marker2_x = ros::param::param("marker2_x", 30.1752);
    marker2_y = ros::param::param("marker2_y", 0);
    lidar_x = ros::param::param("lidar_x", 23.1752);
    lidar_y = ros::param::param("lidar_y", -2.5);
    lidar_yaw = ros::param::param("lidar_yaw", 0);
    in_real_machine = ros::param::param("in_real_machine", true);

    ros::Subscriber pointCLoudSub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_pc_topic, 100, CalibrateCallback);
    pub_line_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/line_cloud", 100, true);
    pub_check_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/check_cloud", 100, true);

    // ros::spin();
    while (ros::ok())
    {
        ros::spinOnce();

        if (calibrate_flag)
        {
            std::cout << "transform_matrix_ave: " << std::endl
                      << transform_matrix_ave.matrix() << std::endl;
            break;
        }
    }

    Eigen::Matrix4d tf_matrix = transform_matrix_ave.matrix();
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            tf_params.push_back(tf_matrix(i, j));
            cout << tf_params[tf_params.size() - 1] << endl;;
        }
    }
    ros::param::set("lidar_tf_params", tf_params);
    ros::param::set("lidar_calibrate_flag", calibrate_flag);

    Eigen::Isometry3d test_tf;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            test_tf(i, j) = tf_params[i*4 + j];
        }
    }
    std::cout << "test_tf: " << std::endl
              << test_tf.matrix() << std::endl;

    return 0;
}
