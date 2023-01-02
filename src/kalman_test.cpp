#include <iostream>
#include <algorithm>
#include <random>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "kf_filter/kalman.h"

using namespace std;

geometry_msgs::PoseStamped truePose;
nav_msgs::Path truePath;
ros::Publisher pub_true_path;

geometry_msgs::PoseStamped detectPose;
nav_msgs::Path detectPath;
ros::Publisher pub_detect_path;

geometry_msgs::PoseStamped filteredPose;
nav_msgs::Path filteredPath;
ros::Publisher pub_filtered_path;

const double dt = 0.1, total_t = 20.0;
Eigen::VectorXd x;
Eigen::MatrixXd P0;
Eigen::MatrixXd Q;
Eigen::MatrixXd R;
Eigen::MatrixXd A;
Eigen::MatrixXd B;
Eigen::MatrixXd H;
Eigen::VectorXd z0;
Eigen::VectorXd u_v;
Kalman ka(9, 3, 9);

vector<double> rnd_arr;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kal_test");
    ros::NodeHandle nh;

    pub_true_path = nh.advertise<nav_msgs::Path>("/true_path", 100, true);
    pub_detect_path = nh.advertise<nav_msgs::Path>("/detect_path", 100, true);
    pub_filtered_path = nh.advertise<nav_msgs::Path>("/filtered_path", 100, true);

    truePose.header.frame_id = "map";
    truePath.header.frame_id = "map";
    detectPose.header.frame_id = "map";
    detectPath.header.frame_id = "map";
    filteredPose.header.frame_id = "map";
    filteredPath.header.frame_id = "map";

    ros::Rate r(10);
    double x_true, y_true, z_true;
    double x_obser, y_obser, z_obser;
    double x_res, y_res, z_res;

    x.resize(9);
    x.setZero();
    x(2) = 1;
    x(5) = 0.6;

    P0.resize(9, 9);
    P0.setIdentity();

    Q.resize(9, 9);
    Q.setIdentity();
    Q(7,7) = Q(8,8) = Q(6,6) = Q(2,2) = 0.0001;
    Q(5,5) = 0.001;
    Q(1,1) = 0.001;
    Q(0,0) = 0.01;
    Q(4,4) = 0.1;

    R.resize(3, 3);
    R.setIdentity();
    R(2,2) = 10;

    A.resize(9, 9);
    A.setIdentity();
    A(0, 1) = A(3, 4) = A(6, 7) = A(1, 2) = A(4, 5) = A(7, 8) = dt;
    A(0, 2) = A(3, 5) = A(6, 8) = dt * dt / 2;

    B.resize(9, 9);
    B.setIdentity();

    H.resize(3, 9);
    H.setZero();
    H(0, 0) = H(1, 3) = H(2, 6) = 1;

    z0.resize(3);

    u_v.resize(9, 1);
    u_v.setZero();

    ka.Init_Par(x, P0, R, Q, A, B, H, u_v);

    srand((int)time(0));
    for(int i = 0; i < 500; i++){
        // default_random_engine e1(time(0));
        // default_random_engine e1(time(0));
        // default_random_engine e1; e1.seed(32767); std::default_random_engine e2(32767);
        // normal_distribution<double> distribution(0.0, 0.1);
        
        int rnd = rand()%100 - 50;
        double rnd_i = (double)rnd/250.0;
        cout<<rnd_i<<" round "<<i<<endl;
        rnd_arr.push_back(rnd_i);
    }

    for (double t = 0; t < total_t; t = t + dt)
    {
        x_true = t + 0.008*t*t + 0.3*sqrt(0.7*t);
        // y_true = 4 * sin(x_true / 2);
        y_true = 0.6*t - 0.25*sqrt(t) + 0.8 * sin(x_true / 2);
        z_true = 0;
        truePose.header.stamp = ros::Time::now();
        truePose.pose.position.x = x_true;
        truePose.pose.position.y = y_true;
        truePose.pose.position.z = z_true;
        truePath.header.stamp = truePose.header.stamp;
        truePath.poses.push_back(truePose);

        
        double rnd = 0.5*rnd_arr[t/dt];
        cout << rnd << endl;
        x_obser = x_true - 1.4*rnd + 1.2*rnd*rnd;
        y_obser = y_true + 1.5*rnd - 1.7*rnd*rnd;
        z_obser = z_true + rnd;
        detectPose.header.stamp = ros::Time::now();
        detectPose.pose.position.x = x_obser;
        detectPose.pose.position.y = y_obser;
        detectPose.pose.position.z = z_obser;
        detectPath.header.stamp = detectPose.header.stamp;
        detectPath.poses.push_back(detectPose);

        z0(0) = x_obser;
        z0(1) = y_obser;
        z0(2) = z_obser;
        cout << "the " << (t/dt + 1) << " th time predict" << endl;
        ka.Predict_State();
        ka.Predict_Cov();
        ka.Mea_Resd(z0);
        ka.Cal_Gain();
        ka.Update_State();
        ka.Update_Cov();
        x_res = ka.m_x[0];
        y_res = ka.m_x[3];
        z_res = ka.m_x[6];
        filteredPose.header.stamp = ros::Time::now();
        filteredPose.pose.position.x = x_res;
        filteredPose.pose.position.y = y_res;
        filteredPose.pose.position.z = z_res;
        filteredPath.header.stamp = filteredPose.header.stamp;
        filteredPath.poses.push_back(filteredPose);

        pub_true_path.publish(truePath);
        pub_detect_path.publish(detectPath);
        pub_filtered_path.publish(filteredPath);

        r.sleep();
    }

    return 0;
}
