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

#if !defined(LOCOMOTION_H)
#define LOCOMOTION_H

#include "math_utils.h"
#include "looper.h"

namespace Locomotion {

template<typename T>
class _Locomotion : public Looper {
protected:
	_Quaternion<T> targetOrientation;
	_Quaternion<T> currentOrientation;

	_Quaternion<T> targetThrust;
	_Quaternion<T> currentThrust;

	timestamp_t thrust_off_timeout;

public:
	_Locomotion() : Looper() {
		thrust_off_timeout = 2000;
	}

	virtual void begin(timestamp_t now) {
		setCurrentOrientation(Quaternion_ZERO);
		setOrientation(Quaternion_ZERO);
		setCurrentThrust(Quaternion_ZERO);
		setThrust(Quaternion_ZERO);
		Looper::begin(now);
	}
	virtual void loop(timestamp_t now)
	{
		if (thrust_off_timeout > 0 && elapsed(now) > thrust_off_timeout) {
			setThrust(Quaternion_ZERO);
		}
		Looper::loop(now);
	}

	virtual void updated(timestamp_t now, timestamp_t timeout) {
		Looper::updated(now);
		thrust_off_timeout = timeout;
	}

	virtual void setOrientation(const _Quaternion<T>& orientation) {
		targetOrientation = orientation;
	}
	virtual const _Quaternion<T>& getOrientation() const {
		return currentOrientation;
	}
	virtual const _Quaternion<T>& getTargetOrientation() const {
		return targetOrientation;
	}

	virtual void setThrust(const _Quaternion<T>& thrust)
	{
		targetThrust = thrust;
	}
	virtual const _Quaternion<T>& getThrust() const
	{
		return currentThrust;
	}
	virtual const _Quaternion<T>& getTargetThrust() const
	{
		return targetThrust;
	}

protected:
	virtual void setCurrentOrientation(const _Quaternion<T>& orientation) {
		currentOrientation = orientation;
	}

	virtual void setCurrentThrust(const _Quaternion<T>& thrust)
	{
		currentThrust = thrust;
	}

};

typedef _Locomotion<real_t> Locomotion;

}

#endif
