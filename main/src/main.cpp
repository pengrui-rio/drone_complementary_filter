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
#include <sstream>

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
  agstf_data.pose.orientation.w = msg->pose.orientation.w;
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
  ekf_data.pose.orientation.w = msg->pose.orientation.w;
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
  eskf_data.pose.orientation.w = msg->pose.orientation.w;
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
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::Rate rate(1000);
 
    ros::Publisher  comp_pub   = nh.advertise<geometry_msgs::PoseStamped>("comparison", 10);
    ros::Publisher  GT_pub     = nh.advertise<geometry_msgs::PoseStamped>("GT_pose_publish", 10);
    ros::Publisher  AGSTF_pub  = nh.advertise<geometry_msgs::PoseStamped>("AGSTF_pose_publish", 10);
    ros::Publisher  EKF_pub    = nh.advertise<geometry_msgs::PoseStamped>("EKF_pose_publish", 10);
    ros::Publisher  ESKF_pub   = nh.advertise<geometry_msgs::PoseStamped>("ESKF_pose_publish", 10);

    ros::Subscriber GT_sub     = nh.subscribe<geometry_msgs::PoseStamped>("GT_pose",    10, gt_sub); 
    ros::Subscriber AGSTF_sub  = nh.subscribe<geometry_msgs::PoseStamped>("AGSTF_pose", 10, agstf_sub); 
    ros::Subscriber EKF_sub    = nh.subscribe<geometry_msgs::PoseStamped>("EKF_pose",   10, ekf_sub); 
    ros::Subscriber ESKF_sub   = nh.subscribe<geometry_msgs::PoseStamped>("ESKF_pose",  10, eskf_sub); 

    geometry_msgs::PoseStamped pose;

    ofstream outFile_yaw, outFile_pit, outFile_rol, outFile_spd;
    outFile_yaw.open("/home/rio/Documents/arclab/18.validation/csv/1.csv",     ios::out); // 打开模式可省略
    outFile_pit.open("/home/rio/Documents/arclab/18.validation/csv/2.csv",     ios::out); // 打开模式可省略
    outFile_rol.open("/home/rio/Documents/arclab/18.validation/csv/3.csv",     ios::out); // 打开模式可省略
    outFile_spd.open("/home/rio/Documents/arclab/18.validation/csv/speed.csv", ios::out); // 打开模式可省略

    ros::Time start_run = ros::Time::now();

    float run_count = 0;
    while(ros::ok())
    {
      if(gt_data_good && agstf_data_good && ekf_data_good && eskf_data_good)
      {
        run_count++;

        pose.pose.orientation.x = gt_data.pose.orientation.x;
        pose.pose.orientation.y = gt_data.pose.orientation.y;
        pose.pose.orientation.z = gt_data.pose.orientation.z;
        pose.header.stamp = ros::Time::now ();
        GT_pub.publish(pose);

        pose.pose.orientation.x = agstf_data.pose.orientation.x;
        pose.pose.orientation.y = agstf_data.pose.orientation.y;
        pose.pose.orientation.z = agstf_data.pose.orientation.z;
        pose.header.stamp = ros::Time::now ();
        AGSTF_pub.publish(pose);

        pose.pose.orientation.x = ekf_data.pose.orientation.x;
        pose.pose.orientation.y = ekf_data.pose.orientation.y;
        pose.pose.orientation.z = ekf_data.pose.orientation.z;
        pose.header.stamp = ros::Time::now ();
        EKF_pub.publish(pose);

        pose.pose.orientation.x = eskf_data.pose.orientation.x;
        pose.pose.orientation.y = eskf_data.pose.orientation.y;
        pose.pose.orientation.z = eskf_data.pose.orientation.z;
        pose.header.stamp = ros::Time::now();
        ESKF_pub.publish(pose);

        // pose.pose.orientation.w = gt_data.pose.orientation.x;
        // pose.pose.orientation.x = agstf_data.pose.orientation.x;
        // pose.pose.orientation.y = ekf_data.pose.orientation.x;
        // pose.pose.orientation.z = eskf_data.pose.orientation.x;

        // pose.pose.orientation.w = gt_data.pose.orientation.y;
        // pose.pose.orientation.x = agstf_data.pose.orientation.y;
        // pose.pose.orientation.y = ekf_data.pose.orientation.y;
        // pose.pose.orientation.z = eskf_data.pose.orientation.y;

        // pose.pose.orientation.w = gt_data.pose.orientation.z;
        // pose.pose.orientation.w = agstf_data.pose.orientation.z;
        // pose.pose.orientation.w = ekf_data.pose.orientation.z;
        // pose.pose.orientation.w = eskf_data.pose.orientation.z;
        
        // pose.pose.orientation.x = agstf_data.pose.orientation.w;
        // pose.pose.orientation.y = ekf_data.pose.orientation.w;
        // pose.pose.orientation.z = eskf_data.pose.orientation.w;
        // pose.header.stamp = ros::Time::now();
        // comp_pub.publish(pose);

        // double time_s = ros::Time::now().toSec() - start_run.toSec();
        // ROS_INFO("time_s: %f  ", time_s);

        // outFile_yaw << time_s << ','
        //             << gt_data.pose.orientation.x  << ',' 
        //             << agstf_data.pose.orientation.x - 160 << ',' 
        //             << ekf_data.pose.orientation.x - 160 << ','
        //             << eskf_data.pose.orientation.x - 160 << endl;

        // outFile_pit << time_s << ','
        //             << gt_data.pose.orientation.y << ',' 
        //             << agstf_data.pose.orientation.y << ',' 
        //             << ekf_data.pose.orientation.y << ','
        //             << eskf_data.pose.orientation.y << endl;

        // outFile_rol << time_s << ','
        //             << gt_data.pose.orientation.z << ',' 
        //             << agstf_data.pose.orientation.z << ',' 
        //             << ekf_data.pose.orientation.z << ','
        //             << eskf_data.pose.orientation.z << endl;

        // outFile_spd << run_count << ','
        //             << agstf_data.pose.orientation.w << ',' 
        //             << ekf_data.pose.orientation.w << ','
        //             << eskf_data.pose.orientation.w << endl;

        gt_data_good    = false;
        agstf_data_good = false;
        ekf_data_good   = false;
        eskf_data_good  = false;
      }

      ros::spinOnce();
      rate.sleep();
    }


    return 0;
}

// 22s motors start