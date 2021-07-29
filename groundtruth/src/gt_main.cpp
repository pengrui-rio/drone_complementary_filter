/**
 * Copyright (c) 2019, Peng LU, ArcLab, Hong Kong PolyU, China
 * peng.lu@polyu.edu.hk
 * @file fixed_waypoint.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

// #include "EKF_Attitude.h"
#include <ros/ros.h>
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

 
bool gt_data_good = false;  

geometry_msgs::TransformStamped gt_data;
void gt_sub(const geometry_msgs::TransformStamped::ConstPtr& msg) 
{
  // ROS_INFO("I heard the pose from the robot"); 
  // ROS_INFO("the position(x,y,z) is %f , %f, %f", msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
  // ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);

  gt_data.transform.translation.x = msg->transform.translation.x;
  gt_data.transform.translation.y = msg->transform.translation.y;
  gt_data.transform.translation.z = msg->transform.translation.z;
  gt_data.transform.rotation.x = msg->transform.rotation.x;
  gt_data.transform.rotation.y = msg->transform.rotation.y;
  gt_data.transform.rotation.z = msg->transform.rotation.z;
  gt_data.transform.rotation.w = msg->transform.rotation.w;

  gt_data_good = true;
}

void quaterinion2euler(double qw, double qx, double qy, double qz, double *Yaw, double *Pitch, double *Roll)
{
	*Yaw    = atan2f((2 * (qx * qy + qw*qz)), (qw*qw + qx*qx - qy*qy - qz*qz) ) ;// yaw
	*Pitch  = asinf(  -2 * qx * qz + 2 * qw* qy) ; // pitch
	*Roll   = atan2f(  2 * qy * qz + 2 * qw * qx, -2 * qx * qx - 2 * qy* qy + 1) ; // roll;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gt");
    ros::NodeHandle nh;
    ros::Rate rate(1000);
    ros::Time last_request = ros::Time::now();
 
    ros::Publisher  atti_pub   = nh.advertise<geometry_msgs::PoseStamped>("GT_pose", 10);
    ros::Subscriber GT_sub     = nh.subscribe("vicon/pengrui01/pengrui01", 10, gt_sub); 

    geometry_msgs::PoseStamped pose;

    while(ros::ok())
    {
      if(gt_data_good)
      {
        // Eigen::Quaterniond q_euler_ESKF(gt_data.transform.rotation.w,
        //                                 gt_data.transform.rotation.x, 
        //                                 gt_data.transform.rotation.y, 
        //                                 gt_data.transform.rotation.z); //w x y z

        // Eigen::Vector3d eulerAngle_ESKF ;//= q_euler_ESKF.matrix().eulerAngles(2, 1, 0);

        double yaw = 0, pitch = 0, roll = 0;
        quaterinion2euler(gt_data.transform.rotation.w,
                          gt_data.transform.rotation.x,
                          gt_data.transform.rotation.y,
                          gt_data.transform.rotation.z,
                          &yaw,
                          &pitch,
                          &roll);

        pose.pose.orientation.x = -yaw * 180.0 / M_PI;
        pose.pose.orientation.y = pitch * 180.0 / M_PI;
        pose.pose.orientation.z = roll * 180.0 / M_PI;
        atti_pub.publish(pose);


        gt_data_good = false;
      }

      ros::spinOnce();
      rate.sleep();
    }


    return 0;
}

// 22s motors start