#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> // 拟合直线
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/point_cloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

// 激光雷达参数(hesai-64)
string lidar_frame_id = "Pandar64", lidar_pc_topic = "/hesai/pandar";
const int vertical_num = 64, horizon_num = 1800;
const float vertical_accuracy = 0.167, horizon_accuracy = 0.2;
const float min_angle = -25, max_angle = 15;
const float ANG = 57.2957795; // 弧度制转角度的比例因数

pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr lane_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr check_cloud(new pcl::PointCloud<pcl::PointXYZI>);
vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ransac_clouds;
ros::Publisher pub_line_cloud;
ros::Publisher pub_check_cloud;
sensor_msgs::PointCloud2 line_cloud_pub;
sensor_msgs::PointCloud2 check_cloud_pub;

Eigen::Isometry3d transform_matrix;

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

void vertical_angle_filter(double target_angle){
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_points(new pcl::PointCloud<pcl::PointXYZI>);

    vector<vector<int>> idx_of_cols(horizon_num);
    for(int idx = 0; idx < input_cloud->points.size(); idx++){
        pcl::PointXYZI pt = input_cloud->points[idx];
        int col_i = ((horizon_num*horizon_accuracy/2) - atan2(pt.y, pt.x) * ANG) / horizon_accuracy;
        idx_of_cols[col_i].push_back(idx);
    }

    for(int col = 0; col < idx_of_cols.size(); col++){
        double min_dist = 420;
        pcl::PointXYZI target_pt;

        for(int i = 0; i < idx_of_cols[col].size(); i++){
            int idx = idx_of_cols[col][i];
            pcl::PointXYZI pt = input_cloud->points[idx];
            double verticalAngle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
            double angle_dist = abs(verticalAngle - target_angle);
            if(angle_dist < min_dist){
                min_dist = angle_dist;
                target_pt = pt;
            }
        }

        if(min_dist < 420){
            target_points->points.push_back(target_pt);
        }
    }
    input_cloud->clear();
    *input_cloud += *target_points;

}

int ransac_line_fitting(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, int maxiter,
                        int consensus_thres, double dis_thres,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_cloud,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr outlier_cloud)
{
    int point_num = in_cloud->size();
    default_random_engine rng;
    rng.seed(10);
    uniform_int_distribution<unsigned> uniform(0, point_num - 1);
    set<int> selectIndexs, consensusIndexs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr selectPoints(new pcl::PointCloud<pcl::PointXYZI>);
    int isNonFind = 1;
    int bestconsensus_num = 0;
    int iter = 0;
    double tmp_A, tmp_B, tmp_C;
    while (iter < maxiter)
    {
        selectIndexs.clear();
        selectPoints->clear();
        while (1)
        {
            int index = uniform(rng);
            selectIndexs.insert(index);
            if (selectIndexs.size() == 2) // 2 samples
            {
                break;
            }
        }
        set<int>::iterator selectiter = selectIndexs.begin();
        while (selectiter != selectIndexs.end())
        {
            int index = *selectiter;
            selectPoints->push_back(in_cloud->points[index]);
            selectiter++;
        }
        // 这里是直线拟合，也可以换成多次曲线拟合
        double deltaY = selectPoints->points[1].y - selectPoints->points[0].y;
        double deltaX = selectPoints->points[1].x - selectPoints->points[0].x;
        tmp_A = deltaY;
        tmp_B = -deltaX;
        tmp_C = -deltaY * selectPoints->points[1].x + deltaX * selectPoints->points[1].y;
        int dataiter = 0;
        double meanError = 0;
        set<int> tmpConsensusIndexs;
        std::vector<int> inlierIndex, outlierIndex;
        while (dataiter < point_num)
        {
            double dist = (tmp_A * in_cloud->points[dataiter].x + tmp_B * in_cloud->points[dataiter].y + tmp_C) / sqrt(tmp_A * tmp_A + tmp_B * tmp_B);
            dist = dist > 0 ? dist : -dist;
            if (dist <= dis_thres)
            {
                tmpConsensusIndexs.insert(dataiter);
                inlierIndex.push_back(dataiter);
            }
            else
            {
                outlierIndex.push_back(dataiter);
            }
            meanError += dist;
            dataiter++;
        }
        if (tmpConsensusIndexs.size() >= consensus_thres && tmpConsensusIndexs.size() >= bestconsensus_num)
        {
            bestconsensus_num = consensusIndexs.size();
            consensusIndexs = tmpConsensusIndexs;
            isNonFind = 0;
            pcl::copyPointCloud(*in_cloud, inlierIndex, *inlier_cloud);
            pcl::copyPointCloud(*in_cloud, outlierIndex, *outlier_cloud);
        }
        iter++;
        inlierIndex.clear();
        outlierIndex.clear();
    }
    return isNonFind;
}

int line_detect(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud)
{
    int maxiter = 700;
    int consensus_thres = 5;
    double dis_thres = 0.02;
    int intensity = 0;
    while (in_cloud->size() > 5)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_points(new pcl::PointCloud<pcl::PointXYZI>),
        outlier_points(new pcl::PointCloud<pcl::PointXYZI>);
        if (!ransac_line_fitting(in_cloud, maxiter, consensus_thres, dis_thres, inlier_points, outlier_points))
        {
            for (int idx = 0; idx < inlier_points->points.size(); idx++)
            {
                inlier_points->points[idx].intensity = intensity;
            }
            *lane_cloud += *inlier_points;
            ransac_clouds.push_back(inlier_points);
            intensity++;
        }
        if (outlier_points->size() == 0)
        {
            break;
        }
        in_cloud->clear();
        *in_cloud = *outlier_points;
        outlier_points->clear();
    }
    int size = ransac_clouds.size();
    return size;
}

Eigen::Vector2d get_intersection(Eigen::Vector4d lineParam1, Eigen::Vector4d lineParam2){
    // 求两条平面直线的交点
    // Vec4d :参数的前半部分给出的是直线的方向，而后半部分给出的是直线上的一点
	Eigen::Vector2d result(-1, -1);

	double cos_theta = lineParam1[0];
	double sin_theta = lineParam1[1];
	double x = lineParam1[2];
	double y = lineParam1[3];
	double k = sin_theta / cos_theta;
	double b = y - k * x;

	cos_theta = lineParam2[0];
	sin_theta = lineParam2[1];
	x = lineParam2[2];
    y = lineParam2[3];
	double k1 = sin_theta / cos_theta;
	double b1 = y - k1 * x;

	result[0] = (b1 - b) / (k - k1);
	result[1] = k * result[0] + b;

	return result;
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

Eigen::Isometry3d get_rotation(Eigen::Vector3d vectorBefore, Eigen::Vector3d vectorAfter){
    Eigen::Matrix3d rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();

    Eigen::Isometry3d homo_tf = Eigen::Isometry3d::Identity();
    homo_tf.rotate(rotMatrix);
    
    return homo_tf;
}

Eigen::Isometry3d get_translation(Eigen::Isometry3d rot_tf, Eigen::Vector3d vectorBefore, Eigen::Vector3d vectorAfter){
    Eigen::Vector3d temp_vector = rot_tf*vectorBefore;
    Eigen::Vector3d translation(vectorAfter[0] - vectorBefore[0],
                                vectorAfter[1] - vectorBefore[1],
                                vectorAfter[2] - vectorBefore[2]);
    Eigen::Isometry3d homo_tf = rot_tf;
    homo_tf.pretranslate(translation);
    
    return homo_tf;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr points_transform(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Isometry3d tf){
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_points(new pcl::PointCloud<pcl::PointXYZI>);
    *transformed_points += *point_cloud;
    for(int i = 0; i < transformed_points->points.size(); i++){
        Eigen::Vector3d temp_point(transformed_points->points[i].x, transformed_points->points[i].y, transformed_points->points[i].z);
        Eigen::Vector3d point_transformed = tf*temp_point;
        transformed_points->points[i].x = point_transformed[0];
        transformed_points->points[i].y = point_transformed[1];
        transformed_points->points[i].z = point_transformed[2];
    }
    return transformed_points;
}

void calibrate()
{
    Eigen::Vector3d central_ys;
    for (int i = 0; i < ransac_clouds.size(); i++)
    {
        pcl::PointXYZ centrol_pose = get_central_pose(ransac_clouds[i]);
        central_ys[i] = centrol_pose.y;
    }
    Eigen::Vector3d::Index maxCol, minCol;
	double min = central_ys.minCoeff(&minCol);
	double max = central_ys.maxCoeff(&maxCol);
	cout << "Max = \n" << max << endl;
	cout << "Min = \n" << min << endl;
	cout << "minCol = " << minCol << "maxCol = " << maxCol << endl;

    vector<Eigen::Vector4d> lineParams(3);
    for (int i = 0; i < ransac_clouds.size(); i++)
    {
        for (int pt_idx = 0; pt_idx < ransac_clouds[i]->points.size(); pt_idx++)
        {
            ransac_clouds[i]->points[pt_idx].z = 0;
        }
        pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(ransac_clouds[i]));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_line);
        ransac.setDistanceThreshold(0.02); // 内点到模型的最大距离
        ransac.setMaxIterations(1000);     // 最大迭代次数
        ransac.computeModel();             // 直线拟合
        //--------------------------根据索引提取内点------------------------
        vector<int> inliers;
        ransac.getInliers(inliers);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud<pcl::PointXYZI>(*ransac_clouds[i], inliers, *cloud_line);
        //----------------------------输出模型参数--------------------------
        Eigen::VectorXf coef;
        ransac.getModelCoefficients(coef);

        ROS_INFO("ransac round %d", i);
        cout << "直线方程为："
             << "   (x - " << coef[0] << ") / " << coef[3]
             << " = (y - " << coef[1] << ") / " << coef[4]
             << " = (z - " << coef[2] << ") / " << coef[5] 
             << " size: " << ransac_clouds[i]->points.size() << endl;
        Eigen::Vector4d lineParam;
        lineParam << coef[3], coef[4], coef[0], coef[1];
        if(i == minCol){lineParams[0] = lineParam;}
        else if(i == maxCol){lineParams[2] = lineParam;}
        else{lineParams[1] = lineParam;}
    }

    vector<Eigen::Vector2d> intersections(3);
    intersections[0] = get_intersection(lineParams[1], lineParams[0]);
    intersections[1] = get_intersection(lineParams[1], lineParams[2]);
    ROS_INFO("intersections are (%f, %f),(%f, %f);", intersections[0][0], intersections[0][1],
                                                     intersections[1][0], intersections[1][1]);

    Eigen::Vector3d vectorBefore(intersections[1][0] - intersections[0][0], 
                                 intersections[1][1] - intersections[0][1], 0), 
                    vectorAfter(0, 1, 0);
    transform_matrix = get_rotation(vectorBefore, vectorAfter);
    vectorBefore << intersections[0][0], intersections[0][1], 0;
    vectorAfter << 11.888, 0, 0;
    transform_matrix = get_translation(transform_matrix, vectorBefore, vectorAfter);
    std::cout << "transform_matrix: " << std::endl
              << transform_matrix.matrix() << std::endl;
}

void CalibrateCallback(const sensor_msgs::PointCloud2::ConstPtr &point_msg)
{
    ROS_INFO("round begin======");

    pcl::fromROSMsg(*point_msg, *input_cloud);

    int size;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *filtered_cloud += *(position_filter(-7 - 0.5, 4 + 0.5, 0));
    *filtered_cloud += *(position_filter(-7 - 0.5, 4 + 4.950 + 0.5, 0));
    input_cloud->clear();
    *input_cloud += *filtered_cloud;

    vertical_angle_filter(0);
    size = input_cloud->points.size();
    ROS_INFO("size of input_cloud is %d", size);

    int line_num = line_detect(input_cloud);
    ROS_INFO("size of lines is %d", line_num);
    pcl::toROSMsg(*lane_cloud, line_cloud_pub);
    // pcl::toROSMsg(*ransac_clouds[1], line_cloud_pub);
    line_cloud_pub.header.stamp = ros::Time::now();
    line_cloud_pub.header.frame_id = "odom";
    pub_line_cloud.publish(line_cloud_pub);

    calibrate();

    pcl::fromROSMsg(*point_msg, *input_cloud);
    check_cloud = points_transform(input_cloud, transform_matrix);
    pcl::toROSMsg(*check_cloud, check_cloud_pub);
    check_cloud_pub.header.stamp = ros::Time::now();
    check_cloud_pub.header.frame_id = "odom";
    pub_check_cloud.publish(check_cloud_pub);

    lane_cloud->clear();
    check_cloud->clear();
    ransac_clouds.clear();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_calibrate");
    ros::NodeHandle nh;

    ros::Subscriber pointCLoudSub = nh.subscribe<sensor_msgs::PointCloud2>("/transform_points", 100, CalibrateCallback);
    pub_line_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/line_cloud", 100, true);
    pub_check_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/check_cloud", 100, true);

    ros::spin();

    Eigen::Matrix3d rotMatrix;
    Eigen::Vector3d vectorBefore(1, 0, 0);
    Eigen::Vector3d vectorAfter(sqrt(2), sqrt(2), 0);

    rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();
    cout << "Transform matrix = \n"
         << rotMatrix.matrix() << endl;

    Eigen::Vector3d axi1Before(1, 1, 0);
    Eigen::Vector3d axi1After(0, 0, 0);
    axi1After = rotMatrix * axi1Before;
    cout << "(1,1,0) after rotation = " << axi1After.transpose() << endl;

    Eigen::Vector3d t(0, 1, 0);
    Eigen::Isometry3d homo_tf = Eigen::Isometry3d::Identity(); // T_M3是一个4x4的矩阵
    homo_tf.rotate(rotMatrix);
    homo_tf.pretranslate(t);
    std::cout << "homo_tf: " << std::endl
              << homo_tf.matrix() << std::endl;

    return 0;
}
