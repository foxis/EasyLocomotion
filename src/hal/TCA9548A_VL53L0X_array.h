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

#if !defined(TCA9548A_VL53L0X_ARRAY_H)
#define TCA9548A_VL53L0X_ARRAY_H

#include "../devices/TCA9548A.h"
#include "../devices/VL53L0X.h"
#include "rangesensorbase.h"

namespace Locomotion {

//
// Handles an array of VL53L0X sensors connected via TCA9548A i2C mux.
//
// NOTE: yaw/pitch values must be already filled inside Measurement struct.
//
template<typename T>
class _TCA9548A_VL53L0X_Array : public _RangeSensorBase<T> {
	TCA9548A *mux;
	_VL53L0X<T> *sensors;
	size_t mux_num;
	size_t sensor_num;
	byte last_mux_id;
	byte last_channel_id;

public:
	_TCA9548A_VL53L0X_Array(TCA9548A * mux_arr, size_t mux_num, _VL53L0X<T> * sensors_arr, size_t sensor_num)
	{
		mux = mux_arr;
		sensors = sensors_arr;
		this->mux_num = mux_num;
		this->sensor_num = sensor_num;
		last_mux_id = 255;
		last_channel_id = 255;
	}

  virtual void begin(bool init) {
		for (size_t i = 0; i < mux_num; i++) {
			mux[i].begin(init);
		}
		for (size_t i = 0; i < sensor_num; i++) {
			if (init) set_channel(i);
			sensors[i].begin(init);
		}
  }

	virtual bool reset() {
		for (size_t i = 0; i < mux_num; i++) {
			mux[i].reset();
		}
		for (size_t i = 0; i < sensor_num; i++) {
			set_channel(i);
			if (!sensors[i].reset())
				return false;
		}
		return true;
	}

	void test(bool * results) {
		for (size_t i = 0; i < mux_num; i++) {
			*(results++) = mux[i].test();
		}
		for (size_t i = 0; i < sensor_num; i++) {
			set_channel(i);
			*(results++) = sensors[i].test();
		}
	}

	void setIOTimeout(unsigned long io_timeout_us) {
		for (size_t i = 0; i < sensor_num; i++) {
			sensors[i].setIOTimeout(io_timeout_us);
		}
	}


	virtual T getMaxDistance() { return 2000; }
	virtual T getMinDistance() { return 0; }
	virtual T getMinYaw() { return -.5 * 25.0 * 3.1415 / 180.0; }
	virtual T getMaxYaw() { return .5 * 25.0 * 3.1415 / 180.0; }
	virtual T getMinPitch() { return -.5 * 25.0 * 3.1415 / 180.0; }
	virtual T getMaxPitch() { return .5 * 25.0 * 3.1415 / 180.0; }

	virtual bool startSingleSampling() {
		bool timeout = false;
		for (size_t i = 0; i < sensor_num; i++) {
			set_channel(i);
			timeout |= sensors[i].startSingleSampling();
		}
		return timeout;
	}
	virtual bool startContinuousSampling() {
		bool timeout = false;
		for (size_t i = 0; i < sensor_num; i++) {
			set_channel(i);
			timeout |= sensors[i].startContinuousSampling();
		}
		return timeout;
	}

	virtual bool stopContinuousSampling() {
		bool timeout = false;
		for (size_t i = 0; i < sensor_num; i++) {
			set_channel(i);
			timeout |= sensors[i].stopContinuousSampling();
		}
		return timeout;
	}

	virtual bool readMeasurement(typename _RangeSensorBase<T>::Measurement_t * result, size_t max_readings, size_t offset) {
		bool timeout = false;
		for (size_t i = 0; i < sensor_num && i < max_readings; i++) {
			set_channel(i);
			timeout |= sensors[i].readMeasurement(result, 1, offset + i);
		}
		result->num_readings = min(sensor_num, max_readings);
		result->timeout = timeout;
		return timeout;
	}


private:
	void set_channel(size_t index) {
		byte mux_id = index >> 3;
		byte channel_id = index % 8;

		if (mux_id != last_mux_id)
			for (size_t i = 0; i < mux_num; i++) {
				if (mux_id != i)
					mux[i].setChannelMask(0);
			}
		if (channel_id != last_channel_id) {
			mux[mux_id].setChannel(channel_id);
		}
		last_mux_id = mux_id;
		last_channel_id = channel_id;
	}
};

}

#endif
