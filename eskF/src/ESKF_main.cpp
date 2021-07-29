/**
 * Copyright (c) 2019, Peng LU, ArcLab, Hong Kong PolyU, China
 * peng.lu@polyu.edu.hk
 * @file fixed_waypoint.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "ESKF_Attitude.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

// for recording the file
#include <iostream>
#include <fstream>

// rosbag record -O AHRS.bag /mavros/imu/data /mavros/imu/mag
// scp -r up@192.168.10.218:~/pengrui/AHRS.bag ~/

using namespace std;
using namespace IMU;


bool imu_data_good = false; 
sensor_msgs::Imu imu_data;
void imu_sub(const sensor_msgs::Imu::ConstPtr& data) 
{
	imu_data=*data;

    // ROS_INFO("rec imu_data!!!");
    // cout << "gyrx:" << imu_data.angular_velocity.x << endl;
    // cout << "gyry:" << imu_data.angular_velocity.y << endl;
    // cout << "gyrz:" << imu_data.angular_velocity.z << endl;
    // cout << "accx:" << imu_data.linear_acceleration.x << endl;
    // cout << "accy:" << imu_data.linear_acceleration.y << endl;
    // cout << "accz:" << imu_data.linear_acceleration.z << endl;
    imu_data_good = true;
}

bool mag_data_good = false; 
sensor_msgs::MagneticField mag_data;
void mag_sub(const sensor_msgs::MagneticField::ConstPtr& data)
{
	mag_data=*data;

    // ROS_INFO("rec mag_data!!!");
    // cout << "magx:" << mag_data.magnetic_field.x << endl;
    // cout << "magy:" << mag_data.magnetic_field.y << endl;
    // cout << "magz:" << mag_data.magnetic_field.z << endl;
    mag_data_good = true;
}


void quaterinion2euler(double qw, double qx, double qy, double qz, double *Yaw, double *Pitch, double *Roll)
{
	*Yaw    = atan2f((2 * (qx * qy + qw*qz)), (qw*qw + qx*qx - qy*qy - qz*qz) ) ;// yaw
	*Pitch  = asinf(  -2 * qx * qz + 2 * qw* qy) ; // pitch
	*Roll   = atan2f(  2 * qy * qz + 2 * qw * qx, -2 * qx * qx - 2 * qy* qy + 1) ; // roll;
}




int main(int argc, char **argv)
{

	// define new variables
	double command_x, command_y, command_z;


    ros::init(argc, argv, "ESKF");
    ros::NodeHandle nh;
    ros::Rate rate(1000);
    ros::Time last_time = ros::Time::now();

 
    ros::Publisher  atti_pub = nh.advertise<geometry_msgs::PoseStamped>
                                    ("ESKF_pose", 10);
 
    ros::Subscriber IMU_sub  = nh.subscribe<sensor_msgs::Imu>
		                        	("/mavros/imu/data", 10, imu_sub); 
	ros::Subscriber MAG_sub  = nh.subscribe<sensor_msgs::MagneticField>
			                        ("/mavros/imu/mag",  10, mag_sub);

    Eigen::Matrix<double, 12, 1> ESKF_InitVec;
    ESKF_InitVec << 1e-5*Eigen::Vector3d::Ones(), 1e-9*Eigen::Vector3d::Ones(),
                    1e-3*Eigen::Vector3d::Ones(), 1e-4*Eigen::Vector3d::Ones();
    IMU::ESKF_Attitude ESKF_AHRS(ESKF_InitVec, 0);

	float	_gyro[3];
	float	_accel[3];
	float	_mag[3];

    geometry_msgs::PoseStamped pose;
    while(ros::ok())
    {
        if(imu_data_good && mag_data_good)
        {
            _gyro[0]  = imu_data.angular_velocity.x;
            _gyro[1]  = imu_data.angular_velocity.y;
            _gyro[2]  = imu_data.angular_velocity.z;

            _accel[0] = imu_data.linear_acceleration.x;
            _accel[1] = imu_data.linear_acceleration.y;
            _accel[2] = imu_data.linear_acceleration.z;

            _mag[0] = mag_data.magnetic_field.x;
            _mag[1] = mag_data.magnetic_field.y;
            _mag[2] = mag_data.magnetic_field.z;

            // /////////////////////////////////////////////////////////
            ros::Time now = ros::Time::now();
            ESKF_AHRS.deltaT = double(now.toSec() - last_time.toSec() );
            // cout << "EKF_AHRS.deltaT: " << EKF_AHRS.deltaT << endl;
            last_time = ros::Time::now();



            Vector_9 measurement;
            measurement  << _gyro[0],
                            _gyro[1],
                            _gyro[2],
                            _accel[0],
                            _accel[1],
                            _accel[2],
                            _mag[0],
                            _mag[1],
                            _mag[2];						

            ros::Time start_point = ros::Time::now();

            // for(int i = 0; i < 2; i++)
            // {
                ESKF_AHRS.Read_SensorData(measurement);

                if (ESKF_AHRS.State_Vector.size() == 0 || ESKF_AHRS.quaternion.size() == 0)
                {
                    ESKF_AHRS.Init_Estimator();
                }
                else
                {
                    ESKF_AHRS.NominaState_Predict();
                    ESKF_AHRS.ErrorState_Predict();
                    ESKF_AHRS.Update_Filter();
                    ESKF_AHRS.Update_NomianState();
                    ESKF_AHRS.Reset_ErrorState();
                }
            // }

            ros::Time end_point = ros::Time::now();
            float runtime = double(end_point.toSec() - start_point.toSec() );
            // ROS_INFO("ESKF_AHRS dt %f", ESKF_AHRS.deltaT);



            Eigen::Quaterniond q = ESKF_AHRS.quaternion.back();

            // ESKF_AHRS.Euler = Quaternion_to_Euler(q);

            // cout << "ESKF_AHRS.Euler(0):" << ESKF_AHRS.Euler(0)<< endl;
            // cout << "ESKF_AHRS.Euler(1):" << ESKF_AHRS.Euler(1)<< endl;
            // cout << "ESKF_AHRS.Euler(2):" << ESKF_AHRS.Euler(2)<< endl<< endl;
            
            // pose.pose.position.x = ESKF_AHRS.Euler(0);
            // pose.pose.position.y = ESKF_AHRS.Euler(1);
            // pose.pose.position.z = ESKF_AHRS.Euler(2);
            // atti_pub.publish(pose);

            double yaw = 0, pitch = 0, roll = 0;
            quaterinion2euler(q.w(),
                              q.x(),
                              q.y(),
                              q.z(),
                              &yaw,
                              &pitch,
                              &roll);

            yaw = yaw * 180.0 / M_PI + 160;
            pitch = -pitch * 180.0 / M_PI;
            roll = roll * 180.0 / M_PI;
            if(roll < -90 )
            {
                roll = roll + 180;
            }
            if(roll > 90)
            {
                roll = roll - 180;
            }
            
            pose.pose.orientation.w = runtime;
            pose.pose.orientation.x = yaw   ;
            pose.pose.orientation.y = pitch ;
            pose.pose.orientation.z = roll  ;

            // ROS_INFO("%f, %f, %f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
            atti_pub.publish(pose);


            imu_data_good = false;
            mag_data_good = false;
        }

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
