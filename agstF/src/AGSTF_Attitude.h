#ifndef MAHONY_AHRS_H_
#define MAHONY_AHRS_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "TypeDefs.h"

#include "matrix/math.hpp"


#ifdef _WIN32
#include "windows.h"
#else 
#include <unistd.h>  
#endif

using namespace std;

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;


namespace IMU
{

class AGSTF_Attitude
{
public:

	AGSTF_Attitude(bool flag, double dt);

	bool filter_update(float dt);

	bool init();



// private:
	// //designed_filter start
	// //commander takeoff
	//define 变量
	Vector_9 cur_measurement;

	double Kp = 10, Ki = 1;
	double deltaT = 0.0001;
	// Vector_3 Integ_angular;

	vector< Eigen::Quaterniond > quaternion;

	bool mbStopped;
	bool mbStopRequested;
	std::mutex mMutexStop;

	Eigen::Quaterniond quat_temp;
	Eigen::Quaterniond quat_temp_new;




	////////////////////////////////////
	float twoKp;		// 2 * proportional gain (Kp)
	float twoKi;		// 2 * integral gain (Ki)
	// float q10, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
	float invSampleFreq;
	float roll, pitch, yaw;
	char anglesComputed;



	//////////////////////////////////////////////
	const float _dt_min = 0.00001f;
	const float _dt_max = 0.02f;
	
	float		_w_accel = 0.0f;
	float		_w_mag = 0.0f;
	float		_w_ext_hdg = 0.0f;
	float		_w_gyro_bias = 0.0f;
	float		_mag_decl = 0.0f;
	bool		_mag_decl_auto = false;
	bool		_acc_comp = false;
	float		_bias_max = 0.05f;
	int32_t		_ext_hdg_mode = 0;

	Vector3f	_gyro;
	Vector3f	_accel;
	Vector3f	_mag;

	Vector3f	_vision_hdg;
	Vector3f	_mocap_hdg;

	Quatf		_q;
	Vector3f	_rates;
	Vector3f	_gyro_bias;

	Vector3f	_vel_prev;
	// hrt_abstime	_vel_prev_t = 0;

	Vector3f	_pos_acc;

	bool		_inited = false;
	bool		_data_good = false;
	bool		_ext_hdg_good = false;

};

} //namesapce IMU


#endif