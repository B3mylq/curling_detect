#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <random>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/point_cloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

sensor_msgs::PointCloud2 pub_test_cloud;
pcl::PointCloud<pcl::PointXYZI>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZI>);

void add_line_points(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, pcl::PointXYZI start, pcl::PointXYZI end)
{
    double line_length = pcl::euclideanDistance(start, end);
    double resolution = 0.01, point_num = line_length / resolution;

    pcl::PointXYZI start_to_end;
    start_to_end.x = end.x - start.x;
    start_to_end.y = end.y - start.y;
    start_to_end.z = end.z - start.z;

    for (double idx = 0; idx < point_num; idx++)
    {
        pcl::PointXYZI temp_point;
        temp_point.x = idx / point_num * (end.x - start.x) + start.x;
        temp_point.y = idx / point_num * (end.y - start.y) + start.y;
        temp_point.z = idx / point_num * (end.z - start.z) + start.z;
        point_cloud->points.push_back(temp_point);
    }
    // cout << "last point" << point_cloud->points[point_cloud->points.size() - 1] << endl;
}

void add_noise(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud){
    default_random_engine e1(time(0));
    normal_distribution<double> distribution(0.0, 0.004);
    for(int i = 0; i < point_cloud->points.size(); i++){
        double delta_x = distribution(e1), delta_y = distribution(e1);
        point_cloud->points[i].x += delta_x;
        point_cloud->points[i].y += delta_y;
        // cout << "delta x = " << delta_x << "; delta y = " << delta_y << endl;
    }
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "showline");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("ground_marker", 10);
    ros::Publisher prompt_pub = nh.advertise<visualization_msgs::Marker>("prompt_marker", 10);
    ros::Publisher calibrate_pub = nh.advertise<visualization_msgs::Marker>("calibrate_marker", 10);
    ros::Publisher test_points_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/calibrate_points", 100, true);
    ros::Publisher tramsformed_points_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/transform_points", 100, true);
    visualization_msgs::Marker line_list, prompt_list, calibrate_list;
    ros::Rate r(10);

    line_list.header.frame_id = "odom";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 1;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    prompt_list.header = line_list.header;
    prompt_list.ns = "lines";
    prompt_list.action = visualization_msgs::Marker::ADD;
    prompt_list.pose.orientation.w = 1.0;
    prompt_list.id = 2;
    prompt_list.type = visualization_msgs::Marker::LINE_LIST;
    calibrate_list.header = line_list.header;
    calibrate_list.ns = "lines";
    calibrate_list.action = visualization_msgs::Marker::ADD;
    calibrate_list.pose.orientation.w = 1.0;
    calibrate_list.id = 3;
    calibrate_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.05;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    line_list.lifetime = ros::Duration(0);
    prompt_list.scale.x = 0.05;
    prompt_list.color.g = 1.0;
    prompt_list.color.a = 1.0;
    prompt_list.lifetime = ros::Duration(0);
    calibrate_list.scale.x = 0.05;
    calibrate_list.color.r = 1.0;
    calibrate_list.color.a = 1.0;
    calibrate_list.lifetime = ros::Duration(0);
    // Create the vertices for the points and lines
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    // The line list needs two points for each line
    // 场地总框
    line_list.points.push_back(p);
    p.x += 45.720;
    line_list.points.push_back(p);
    line_list.points.push_back(p);
    p.y += 4.950;
    line_list.points.push_back(p);
    line_list.points.push_back(p);
    p.x -= 45.720;
    line_list.points.push_back(p);
    line_list.points.push_back(p);
    p.y -= 4.950;
    line_list.points.push_back(p);
    // 水平中线
    p.x = 1.829;
    p.y = 4.950 / 2;
    line_list.points.push_back(p);
    p.x = 45.720 - p.x;
    line_list.points.push_back(p);

    p.x = 3.658;
    p.y = 0;
    line_list.points.push_back(p);
    p.y = 4.950;
    line_list.points.push_back(p);
    p.x = 45.720 - p.x;
    line_list.points.push_back(p);
    p.y = 0;
    line_list.points.push_back(p);
    // 圆心线
    p.x = 3.658 + 1.829;
    p.y = 0;
    line_list.points.push_back(p);
    p.y = 4.950;
    line_list.points.push_back(p);
    p.x = 45.720 - p.x;
    line_list.points.push_back(p);
    p.y = 0;
    line_list.points.push_back(p);
    // Hog line
    p.x = 3.658 + 1.829 + 6.401;
    p.y = 0;
    line_list.points.push_back(p);
    p.y = 4.950;
    line_list.points.push_back(p);
    p.x = 45.720 - p.x;
    line_list.points.push_back(p);
    p.y = 0;
    line_list.points.push_back(p);

    // 范围提示框
    geometry_msgs::Point origin;
    double x_offset = 7, y_offset = 4;
    double range = 1;
    origin.z = 0.1;
    origin.y = y_offset + 0.5;
    origin.x = -x_offset - 0.5;
    p = origin; p.x = origin.x + range; p.y = origin.y + range;
    prompt_list.points.push_back(p);
    p.y = origin.y - range;
    prompt_list.points.push_back(p);
    prompt_list.points.push_back(p);
    p.x = origin.x - range;
    prompt_list.points.push_back(p);
    prompt_list.points.push_back(p);
    p.y = origin.y + range;
    prompt_list.points.push_back(p);
    prompt_list.points.push_back(p);
    p.x = origin.x + range;
    prompt_list.points.push_back(p);
    origin.z = 0.1;
    origin.y = y_offset + 4.950 + 0.5;
    origin.x = -x_offset - 0.5;
    p = origin; p.x = origin.x + range; p.y = origin.y + range;
    prompt_list.points.push_back(p);
    p.y = origin.y - range;
    prompt_list.points.push_back(p);
    prompt_list.points.push_back(p);
    p.x = origin.x - range;
    prompt_list.points.push_back(p);
    prompt_list.points.push_back(p);
    p.y = origin.y + range;
    prompt_list.points.push_back(p);
    prompt_list.points.push_back(p);
    p.x = origin.x + range;
    prompt_list.points.push_back(p);

    // 校准提示
    origin.z = 0.1;
    origin.y = y_offset;
    origin.x = -x_offset;
    p = origin; 
    calibrate_list.points.push_back(p);
    p.y = origin.y + range;
    calibrate_list.points.push_back(p);
    p = origin; 
    calibrate_list.points.push_back(p);
    p.x = origin.x - range;
    calibrate_list.points.push_back(p);
    origin.z = 0.1;
    origin.y = y_offset + 4.950;
    origin.x = -x_offset;
    p = origin; 
    calibrate_list.points.push_back(p);
    p.y = origin.y + range;
    calibrate_list.points.push_back(p);
    p = origin; 
    calibrate_list.points.push_back(p);
    p.x = origin.x - range;
    calibrate_list.points.push_back(p);

    pcl::PointXYZI start, end;
    start.x = 1.829 + 3.658 + 6.401;
    start.y = 0;
    start.z = 0.08;
    end.x = start.x - 0.36;
    end.y = start.y;
    end.z = 0.085;
    add_line_points(test_cloud, start, end);
    start.x = 1.829 + 3.658 + 6.401;
    start.y = 0;
    start.z = 0.08;
    end.x = start.x;
    end.y = start.y + 0.39;
    end.z = 0.075;
    add_line_points(test_cloud, start, end);
    start.x = 1.829 + 3.658 + 6.401;
    start.y = 4.950;
    start.z = 0.07;
    end.x = start.x - 0.34;
    end.y = start.y;
    end.z = 0.068;
    add_line_points(test_cloud, start, end);
    start.x = 1.829 + 3.658 + 6.401;
    start.y = 4.950;
    start.z = 0.07;
    end.x = start.x;
    end.y = start.y + 0.33;
    end.z = 0.072;
    add_line_points(test_cloud, start, end);
    add_noise(test_cloud);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.pretranslate(Eigen::Vector3d (-(11.888+x_offset), y_offset, 0));
    pcl::PointCloud<pcl::PointXYZI>::Ptr trasformed_cloud = points_transform(test_cloud, T);

    while (ros::ok())
    {
        marker_pub.publish(line_list);
        prompt_pub.publish(prompt_list);
        calibrate_pub.publish(calibrate_list);

        pcl::toROSMsg(*test_cloud, pub_test_cloud);
        pub_test_cloud.header.stamp = ros::Time::now();
        pub_test_cloud.header.frame_id = "odom";
        test_points_pub.publish(pub_test_cloud);

        pcl::toROSMsg(*trasformed_cloud, pub_test_cloud);
        pub_test_cloud.header.stamp = ros::Time::now();
        pub_test_cloud.header.frame_id = "odom";
        tramsformed_points_pub.publish(pub_test_cloud);

        r.sleep();
    }
}
