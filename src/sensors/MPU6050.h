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

#include <Wire.h>
#define SENSORS_MPU6050_ATTACHED
#include <Sensors.h>
#include "../TwoWireDevice.h"
#include "../MadgwickAHRS/MadgwickAHRS.h"

class MPU6050 : public TwoWireDevice {
	Madgwick filter;
	real_t fps;
	Vector3D acceleration;
	Vector3D gyroscope;
	unsigned long duration;
	unsigned long last_now;

public:
	MPU6050() {}
	MPU6050(TwoWire * wire, uint8_t addr, real_t fps) : TwoWireDevice(addr) {
		this->fps = fps;
		duration = 1000L / fps;
	}

	virtual void begin(bool init) {
		TwoWireDevice::begin(init);
		filter.begin(fps);
		Sensors::initialize();
	}

	virtual void loop(unsigned long now) {
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

	const Vector3D& getAcceleration() const {
		return acceleration;
	}
	const Vector3D& getGyroscope() const {
		return gyroscope;
	}
	Vector3D getRollPitchYaw() const {
		return Vector3D(filter.getRollRadians(), filter.getYawRadians(), filter.getPitchRadians());
	}
	Quaternion getOrientation() const {
		return filter.getQuaternion();
	}
}

#endif
