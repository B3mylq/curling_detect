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
#include "depth_make/make_pointxyzitr.h"
//make_pointxyzitr.h为自定义PCL点云消息pcl::PointCloud<pcl::PointXYZITR>的头文件，该自定义点云消息中的每个点都包括了XYZ坐标、反射强度intensity、时间戳time_stamp和点所在线束ring。
using namespace std;
ros::Publisher pub;

struct Stone{
    double x;
    double y;
};
const float PI = 3.1415926;

double min_range = 1.0;
double max_range = 20.0;
double neighbour_range = 5.0;
double stone_r = 0.3;
double stone_height = 0.3;

double deltaTheta = PI/180;//间隔1度  
double startAngle = 150.0*PI/180;  
double endAngle = PI*2 + PI/6; 
double deltaCenter = 0.01; // 0.01 m ? 


void _StoneDetect(const pcl::PointCloud<pcl::PointXYZITR>& data, vector<Stone>& detection_results,double target_a,
                double target_b, bool use_target ){
    double minA,maxA,minB,maxB;  // circle center in the minA/B maxA/B
    double x,y,z,range;        // points in the cloud
    double a,b;                 // circle center
    double theta;
    if(use_target == true){
        minA = target_a - neighbour_range;
        maxA = target_a + neighbour_range;
        minB = target_b - neighbour_range;
        maxB = target_b + neighbour_range;
    }else{
        minA = 40;
        maxA = -40;
        minB = 40;
        maxB = -40;
        for(const auto pt:data.points){
            if(pt.ring != 0){
                x,y,z = pt.x,pt.y,pt.z;
                range = (float)(sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z));
                if(range > min_range && range < max_range && z < stone_height){
                    // OK, possible points
                for (theta = startAngle; theta < endAngle;theta += deltaTheta)  {  
                    a = x - stone_r*cos(theta);  
                    b = y - stone_r*sin(theta);  
                    if (a > maxA) {  
                        maxA = a;  
                    }else if(a < minA) {  
                        minA = a;  
                    }  
                    if (b > maxB) {  
                        maxB = b;  
                    }else if (b < minB) {  
                        minB = b;  
                    }  
                    }
                }}
        // Now we find the possible center(a,b) : a in [minA,maxA], b in [minB, maxB]
        } 
        } 
    double aScale = maxA - minA;
    double bScale = maxB - minB;
    int aScale_ = int(aScale / deltaCenter);
    int bScale_ = int(bScale / deltaCenter);


    int* VoteBox = new int[aScale_ * bScale_];
    // init
    for(int i = 0;i < aScale_ * bScale_; i++){
        VoteBox[i] = 0;
    }
    // Start Voting 

    int a_id,b_id ; 
    for(const auto pt:data.points){
        x,y,z = pt.x,pt.y,pt.z;
        range = (float)(sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z));
        if(range > min_range && range < max_range && z < stone_height){
            // possible point
            for (theta = startAngle; theta < endAngle;theta += deltaTheta){
                a = x - stone_r*cos(theta);  
                b = y - stone_r*sin(theta);
                if(a > minA && a <maxA &&  b>minB && b<maxB){
                    // OK possible center 
                    a_id = int((a - minA)/deltaCenter);
                    b_id = int((b - minB)/deltaCenter);
                    VoteBox[b_id * aScale_ + a_id ] = VoteBox[b_id * aScale_ + a_id ] + 1 ;
                }
            }

        }
    }
    // Voting End 
    int VoteMax = 0;
    int VoteMaxa_id,VoteMaxb_id;
    for(b_id = 0; b < bScale_ ;b++){
        for(a_id = 0; a < aScale_ ;a++){
            if(VoteBox[b_id * aScale_ + a_id] > VoteMax){
                VoteMax = VoteBox[b_id * aScale_ + a_id];
                VoteMaxa_id = a_id;
                VoteMaxb_id = b_id;
            }
        }
    }
    // Find Results 
    
    for(b_id = 0; b < bScale_ ;b++){
        for(a_id = 0; a < aScale_ ;a++){
            if(VoteBox[b_id * aScale_ + a_id] >= VoteMax){
                a = float((a_id / aScale_) * (maxA - minA) + minA);
                b = float((b_id / bScale_) * (maxB - minB) + minB);
                Stone s = {a,b};
                detection_results.push_back(s);
            }
        }
    }
}


void StoneDetect(const sensor_msgs::PointCloud2::ConstPtr& point_msg){
    pcl::PointCloud<pcl::PointXYZITR> cloud_msg;
    pcl::fromROSMsg(*point_msg, cloud_msg);
    cloud_msg.header.frame_id = "map";
 
    float range; //range为该点的深度值
    int row_i, col_i; 
    int scan_num = 16;
    int horizon_num = 1800;
    const float PI = 3.1415926;
    const float ANG = 57.2957795; //弧度制转角度的比例因数
    const float horizon_accuracy = 0.2;
    
    vector<Stone> detection_results;
    _StoneDetect(cloud_msg,detection_results,0.0,0.0,false);

    pub.publish(cloud_msg);

}


int main (int argc, char **argv)
{
	ros::init (argc, argv, "detection_module");
	ros::NodeHandle nh;
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZITR>>("pcl_points", 100, true);
    ros::Subscriber mapSub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, StoneDetect);
    ros::spin();
    return 0;
}