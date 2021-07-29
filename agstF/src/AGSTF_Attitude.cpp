#include <iostream>
#include "AGSTF_Attitude.h"
#include "Convert.h"

using namespace std;
namespace IMU
{

	AGSTF_Attitude::AGSTF_Attitude(bool flag, double dt)
	{

	}

	bool AGSTF_Attitude::filter_update(float dt)
	{
		if (!_inited) {

			// if (!_data_good) {
			// 	return false;
			// }

			return init();
		}

		Quatf q_last = _q;

		// Angular rate of correction
		Vector3f corr;
		float spinRate = _gyro.length();
		// cout << "spinRate: " << spinRate << endl;

		///// Magnetometer correction
		///// Project mag field vector to global frame and extract XY component
		Vector3f mag_earth = _q.conjugate(_mag);
		float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - 0);
		float gainMult = 1.0f;
		const float fifty_dps = 0.873f;

		if (spinRate > fifty_dps) 
		{
			// gainMult = math::min(spinRate / fifty_dps, 10.0f);
			if(spinRate / fifty_dps < 10.0f)
			{
				gainMult = spinRate / fifty_dps;
			}
			else
			{
				gainMult = 10.0f;
			}
		}

		/////// Project magnetometer correction to body frame
		corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -mag_err)) * 1 * 1;


		_q.normalize();

		// Gyro bias estimation
		if (spinRate < 0.175f) 
		{
			_gyro_bias += corr * (1 * dt);

			for (int i = 0; i < 3; i++) 
			{
				if(_gyro_bias(i) <= -_bias_max )
				{
					_gyro_bias(i) = -_bias_max;
				}
				if(_gyro_bias(i) >= _bias_max )
				{
					_gyro_bias(i) = _bias_max;
				}
			}
		}

		_rates = _gyro;// + _gyro_bias;

		// Feed forward gyro
		corr += _rates;

		// Apply correction to state
		_q += _q.derivative1(corr) * dt;

		// Normalize quaternion
		_q.normalize();

		// if (!(PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
		// 	PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)))) 
		// {
		// 	// Reset quaternion to last good state
		// 	_q = q_last;
		// 	_rates.zero();
		// 	_gyro_bias.zero();
		// 	return false;
		// }

		return true;
	}


	bool AGSTF_Attitude::init()
	{
		// Rotation matrix can be easily constructed from acceleration and mag field vectors
		// 'k' is Earth Z axis (Down) unit vector in body frame
		Vector3f k = -_accel;
		k.normalize();

		// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
		Vector3f i = (_mag - k * (_mag * k));
		i.normalize();

		// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
		Vector3f j = k % i;

		// Fill rotation matrix
		Dcmf R;
		R.setRow(0, i);
		R.setRow(1, j);
		R.setRow(2, k);

		// Convert to quaternion
		_q = R;

		// // Compensate for magnetic declination
		// Quatf decl_rotation = Eulerf(0.0f, 0.0f, 0.0f);
		// _q = _q * decl_rotation;

		_q.normalize();

		// if (PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
		// 	PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)) &&
		// 	_q.length() > 0.95f && _q.length() < 1.05f) {
		// 	_inited = true;

		// } 
		// else 
		// {
		// 	_inited = false;
		// }

		_inited = true;

		return _inited;
	}

}