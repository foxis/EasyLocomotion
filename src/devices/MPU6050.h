/*  EasyLocomotion
 *
 *  Copyright (C) 2018  foxis (Andrius Mikonis <andrius.mikonis@gmail.com>)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#if !defined(MPU6050_H)
#define MPU6050_H

#define SENSORS_MPU6050_ATTACHED
#include <Sensors.h>
#include "../hal/TwoWireDevice.h"
#include "../math_utils/MadgwickAHRS.h"

namespace Locomotion {

///
/// MPU6050 imu device
///
template<typename T>
class _MPU6050 : public TwoWireDevice {
	_Madgwick<T> filter;
	T fps;
	_Vector3D<T> acceleration;
	_Vector3D<T> gyroscope;
	timestamp_t duration;
	timestamp_t last_now;

public:
	MPU6050() {}
	MPU6050(TwoWire * wire, uint8_t addr, T fps) 
		: TwoWireDevice(addr), fps(fps), duration(1000 / fps)
	{
	}

	virtual void begin(bool init) {
		TwoWireDevice::begin(init);
		filter.begin(fps);
		Sensors::initialize();
	}

	virtual void loop(timestamp_t now) {
		if (now - last_now > duration) {
			Accelerometer *acc = Sensors::getAccelerometer();
			Gyroscope *gyro = Sensors::getGyroscope();

			Vector3 a, r;

			if (acc)
				a = acc->getAcceleration();
			if (gyro)
				r = gyro->getRotation();

			filter.updateIMU(r.x, r.y, r.z, a.x, a.y, a.z);
			acceleration.x = a.x;
			acceleration.y = a.y;
			acceleration.z = a.z;
			gyroscope.x = r.x;
			gyroscope.y = r.y;
			gyroscope.z = r.z;
			
			last_now = now;
		}
	}

	const _Vector3D<T>& getAcceleration() const {
		return acceleration;
	}
	const _Vector3D<T>& getGyroscope() const {
		return gyroscope;
	}
	_Vector3D<T> getRollPitchYaw() const {
		return Vector3D(filter.getRollRadians(), filter.getYawRadians(), filter.getPitchRadians());
	}
	_Quaternion<T> getOrientation() const {
		return filter.getQuaternion();
	}
}

typedef _MPU6050<real_t> MPU6050;

}

#endif
