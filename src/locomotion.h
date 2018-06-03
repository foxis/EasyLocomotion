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

namespace Locomotion {

class Locomotion {
private:
	unsigned long last_updated;
	Quaternion targetOrientation;
	Quaternion currentOrientation;

	Quaternion targetThrust;
	Quaternion currentThrust;

public:
	Locomotion() {
		last_updated = 0;
	}

	virtual void begin() {
		setCurrentOrientation(Quaternion_ZERO);
		setOrientation(Quaternion_ZERO);
		setCurrentThrust(Quaternion_ZERO);
		setThrust(Quaternion_ZERO);
	}
	virtual void loop(unsigned long now)
	{
		if (now - last_updated > 2000) {
			setThrust(Quaternion(0,0,0,0));
			last_updated = now;
		}
	}

	virtual void updated(unsigned long now) {
		last_updated = now;
	}

	virtual void setOrientation(const Quaternion& orientation) {
		targetOrientation = orientation;
	}
	virtual const Quaternion& getOrientation() const {
		return currentOrientation;
	}
	virtual const Quaternion& getTargetOrientation() const {
		return targetOrientation;
	}

	virtual void setThrust(const Quaternion& thrust)
	{
		targetThrust = thrust;
	}
	virtual const Quaternion& getThrust() const
	{
		return currentThrust;
	}
	virtual const Quaternion& getTargetThrust() const
	{
		return targetThrust;
	}

protected:
	virtual void setCurrentOrientation(const Quaternion& orientation) {
		currentOrientation = orientation;
	}

	virtual void setCurrentThrust(const Quaternion& thrust)
	{
		currentThrust = thrust;
	}

};

}

#endif
