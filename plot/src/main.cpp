/**
 * Copyright (c) 2019, Peng LU, ArcLab, Hong Kong PolyU, China
 * peng.lu@polyu.edu.hk
 * @file fixed_waypoint.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

// #include "EKF_Attitude.h"
#include <ros/ros.h>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

// for recording the file
#include <iostream>
#include <fstream>

// rosbag record -O AHRS.bag /mavros/imu/data /mavros/imu/mag
// scp -r up@192.168.10.218:~/pengrui/AHRS.bag ~/

using namespace std;
using namespace cv;

 
bool data_good = false;  

bool gt_data_good = false;  
geometry_msgs::PoseStamped gt_data;
void gt_sub(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  gt_data.pose.orientation.x = msg->pose.orientation.x;
  gt_data.pose.orientation.y = msg->pose.orientation.y;
  gt_data.pose.orientation.z = msg->pose.orientation.z;
  // ROS_INFO("gt_data(x,y,z) is %f , %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  gt_data_good = true;
}

bool agstf_data_good = false;  
geometry_msgs::PoseStamped agstf_data;
void agstf_sub(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  agstf_data.pose.orientation.x = msg->pose.orientation.x;
  agstf_data.pose.orientation.y = msg->pose.orientation.y;
  agstf_data.pose.orientation.z = msg->pose.orientation.z;
  // ROS_INFO("agstf_data(x,y,z) is %f , %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  agstf_data_good = true;
}

bool ekf_data_good = false;  
geometry_msgs::PoseStamped ekf_data;
void ekf_sub(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  ekf_data.pose.orientation.x = msg->pose.orientation.x;
  ekf_data.pose.orientation.y = msg->pose.orientation.y;
  ekf_data.pose.orientation.z = msg->pose.orientation.z;
  // ROS_INFO("ekf_data(x,y,z) is %f , %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  ekf_data_good = true;
}

bool eskf_data_good = false;  
geometry_msgs::PoseStamped eskf_data;
void eskf_sub(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  eskf_data.pose.orientation.x = msg->pose.orientation.x;
  eskf_data.pose.orientation.y = msg->pose.orientation.y;
  eskf_data.pose.orientation.z = msg->pose.orientation.z;
  // ROS_INFO("eskf_data(x,y,z) is %f , %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  eskf_data_good = true;  
}

// void quaterinion2euler(double qw, double qx, double qy, double qz, double *Yaw, double *Pitch, double *Roll)
// {
// 	*Yaw    = atan2f((2 * (qx * qy + qw*qz)), (qw*qw + qx*qx - qy*qy - qz*qz) ) ;// yaw
// 	*Pitch  = asinf(  -2 * qx * qz + 2 * qw* qy) ; // pitch
// 	*Roll   = atan2f(  2 * qy * qz + 2 * qw * qx, -2 * qx * qx - 2 * qy* qy + 1) ; // roll;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RMSE");
    ros::NodeHandle nh;
    ros::Rate rate(1000);
    ros::Time last_request = ros::Time::now();
 
    ros::Subscriber GT_sub     = nh.subscribe<geometry_msgs::PoseStamped>("GT_pose",    10, gt_sub); 
    ros::Subscriber AGSTF_sub  = nh.subscribe<geometry_msgs::PoseStamped>("AGSTF_pose", 10, agstf_sub); 
    ros::Subscriber EKF_sub    = nh.subscribe<geometry_msgs::PoseStamped>("EKF_pose",   10, ekf_sub); 
    ros::Subscriber ESKF_sub   = nh.subscribe<geometry_msgs::PoseStamped>("ESKF_pose",  10, eskf_sub); 

    geometry_msgs::PoseStamped pose;

    Point3f RMSE_agstf, deviat_agstf ;//= 0;
    Point3f RMSE_ekf,   deviat_ekf   ;//= 0;
    Point3f RMSE_eskf,  deviat_eskf ;// = 0;
    float   data_count = 0 ;

    while(ros::ok())
    {
      if(gt_data_good && agstf_data_good && ekf_data_good && eskf_data_good)
      {
        data_count++;

        deviat_agstf.x += pow(gt_data.pose.orientation.x - agstf_data.pose.orientation.x, 2);
        deviat_agstf.y += pow(gt_data.pose.orientation.y - agstf_data.pose.orientation.y, 2);
        deviat_agstf.z += pow(gt_data.pose.orientation.z - agstf_data.pose.orientation.z, 2);

        deviat_ekf.x += pow(gt_data.pose.orientation.x - ekf_data.pose.orientation.x, 2);
        deviat_ekf.y += pow(gt_data.pose.orientation.y - ekf_data.pose.orientation.y, 2);
        deviat_ekf.z += pow(gt_data.pose.orientation.z - ekf_data.pose.orientation.z, 2);

        deviat_eskf.x += pow(gt_data.pose.orientation.x - eskf_data.pose.orientation.x, 2);
        deviat_eskf.y += pow(gt_data.pose.orientation.y - eskf_data.pose.orientation.y, 2);
        deviat_eskf.z += pow(gt_data.pose.orientation.z - eskf_data.pose.orientation.z, 2);

        gt_data_good    = false;
        agstf_data_good = false;
        ekf_data_good   = false;
        eskf_data_good  = false;
      }

      RMSE_agstf.x = sqrt(deviat_agstf.x / data_count);
      RMSE_agstf.y = sqrt(deviat_agstf.y / data_count);
      RMSE_agstf.z = sqrt(deviat_agstf.z / data_count);

      RMSE_ekf.x   = sqrt(deviat_ekf.x / data_count);
      RMSE_ekf.y   = sqrt(deviat_ekf.y / data_count);
      RMSE_ekf.z   = sqrt(deviat_ekf.z / data_count);

      RMSE_eskf.x  = sqrt(deviat_eskf.x / data_count);
      RMSE_eskf.y  = sqrt(deviat_eskf.y / data_count);
      RMSE_eskf.z  = sqrt(deviat_eskf.z / data_count);

      ROS_INFO("RMSE_agstf(x,y,z) is %f , %f, %f", RMSE_agstf.x, RMSE_agstf.y, RMSE_agstf.z);
      ROS_INFO("RMSE_ekf(x,y,z) is %f , %f, %f"  , RMSE_ekf.x,   RMSE_ekf.y,   RMSE_ekf.z);
      ROS_INFO("RMSE_eskf(x,y,z) is %f , %f, %f \n" , RMSE_eskf.x,  RMSE_eskf.y,  RMSE_eskf.z);

      ros::spinOnce();
      rate.sleep();
    }


    return 0;
}

// 22s motors start

// 1-twopointsFly   
// [ INFO] [1602573763.506050012]: RMSE_agstf(x,y,z) is 4.567698  , 2.132142,  3.357675
// [ INFO] [1602573763.506140901]: RMSE_ekf(x,y,z)   is 4.848189  , 2.757134,  4.711125
// [ INFO] [1602573763.506170637]: RMSE_eskf(x,y,z)  is 4.560882  , 2.809295,  4.824224 

// 2-twotrianglesFly
// [ INFO] [1602574503.591207997]: RMSE_agstf(x,y,z) is 8.416880  , 2.131137,  3.273814
// [ INFO] [1602574503.591264212]: RMSE_ekf(x,y,z)   is 8.180061  , 2.861676,  3.984060
// [ INFO] [1602574503.591277286]: RMSE_eskf(x,y,z)  is 8.247521  , 2.257559,  4.446207 

// 3-setpointRotation
// [ INFO] [1602574869.187306716]: RMSE_agstf(x,y,z) is 15.250320 , 4.461918,  11.749332
// [ INFO] [1602574869.187582168]: RMSE_ekf(x,y,z)   is 17.186035 , 3.033000,  11.340785
// [ INFO] [1602574869.187600031]: RMSE_eskf(x,y,z)  is 19.551971 , 29.586166, 35.001682 
