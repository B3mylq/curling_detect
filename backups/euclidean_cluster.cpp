#include<pcl/ModelCoefficients.h>
#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/features/normal_3d.h>
#include<pcl/search/kdtree.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/segmentation/extract_clusters.h>

#include<iostream>
#include<algorithm>
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_ros/filters/filter.h>
#include<pcl_ros/point_cloud.h>
 

int main (int argc, char **argv){
    ros::init (argc, argv, "point_cluster");
	ros::NodeHandle nh;

    // 读Pcd文件中的点云数据
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("/home/b3mylq/ROS_ws/pcl_ws/src/depth_make/data/table_scene_lms400.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

    // 创建筛选对象:使用1cm的叶大小向下采样数据集
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

    // 创建平面模型的分割对象，并设置所有参数
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
 
    int nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // 从剩余的云中分割出平面模型
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // 从输入云中提取平面模型内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // 得到与平面相关的点
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // 提取剩下点
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // 为提取目的应用的搜索方法创建KdTree对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);
    /*
    * 在这里，我们创建了一个PointIndices向量，它在一个向量中包含了实际的索引信息。
    * 每个检测到的聚类的索引都保存在这里—请注意，cluster_indices是一个容器，其中包含每个检测到的集群的一个PointIndices实例。
    * 例如cluster_indices[0]包含点云中第一个群集的所有索引。
    */
    std::vector<pcl::PointIndices> cluster_indices;
    // 创建欧几里得集群提取器对象。
    // 需要为setClusterTolerance()函数设置正确的值。
    //如果取一个非常小的值，一个实际对象可能被视为多个集群
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    //此时所有聚类索引都被保存在cluster_indices中，为了分割成不同点云，还需要对cluster_indices进行迭代，将每个聚类单独保存成单独的点云集群。
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices)
        cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "/home/b3mylq/ROS_ws/pcl_ws/src/depth_make/data/pcdData/cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }

    return (0);
}
