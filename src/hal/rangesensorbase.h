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

#if !defined(RANGE_SENSOR_BASE_H)
#define RANGE_SENSOR_BASE_H

#include <Arduino.h>
#include "../math_utils.h"

namespace Locomotion {

template<typename T>
class _RangeSensorBase {
public:
	typedef struct {
		T distance;
		T yaw;
		T pitch;
	} Reading_t;

	typedef struct {
		bool timeout;
		uint8_t status;
		size_t num_readings;
		Reading_t * readings;
	} Measurement_t;

	typedef struct {
		Measurement_t * result;
		size_t max_readings;
		size_t offset;
	} CallbackData_t;

	typedef void (*MeasurementCallback_t)(CallbackData_t * result);

	_RangeSensorBase() : _RangeSensorBase(NULL, NULL) { }
	_RangeSensorBase(MeasurementCallback_t callback, CallbackData_t * callback_data) {
		measurement_callback = callback;
		this->callback_data = callback_data;
	}

	virtual void begin(bool init) = 0;
	virtual bool reset() = 0;

	virtual T getMaxDistance() = 0;
	virtual T getMinDistance() = 0;
	virtual T getMinYaw() = 0;
	virtual T getMaxYaw() = 0;
	virtual T getMinPitch() = 0;
	virtual T getMaxPitch() = 0;

	virtual bool startSingleSampling() = 0;
	virtual bool startContinuousSampling() = 0;
	virtual bool stopContinuousSampling() = 0;
	virtual bool readMeasurement(Measurement_t * result, size_t max_readings, size_t offset) = 0;

	virtual void measurementInterruptHandler() {
		if (measurement_callback == NULL || callback_data == NULL || callback_data->result == NULL)
			return;
		readMeasurement(callback_data->result, callback_data->max_readings, callback_data->offset);
		measurement_callback(callback_data);
	}

private:
	MeasurementCallback_t measurement_callback;
	CallbackData_t * callback_data;
};

typedef _RangeSensorBase<real_t> RangeSensorBase;

}

#endif
