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

#if !defined(DIFFDRIVE_H)
#define DIFFDRIVE_H

#include "Arduino.h"

#include "locomotion.h"
#include "effectors/SimpleHBridge.h"

namespace Locomotion {

class DiffDriveMixin {
public:
	DiffDriveMixin() {}
protected:
	virtual void setSpeed(real_t a, real_t b) = 0;

	/// power - differential drive train power
	/// w - rotational power (normalized wL)
	/// all control parameters range -1.0 ... +1.0
	void setDiffSpeed(real_t power, real_t w)
	{
		real_t a = (2.0 * power + w) / 2.0;
		real_t b = (2.0 * power - w) / 2.0;
		setSpeed(a, b);
	}
};

class DiffDrive : public Locomotion, protected DiffDriveMixin, protected SimpleHBridge
{

  real_t _wheelBase;

public:
  DiffDrive(uint8_t AA, uint8_t AB, uint8_t BA, uint8_t BB, real_t wheelBase)
    :DiffDrive(AA, AB, BA, BB, PWMRANGE+1, wheelBase)
  {

  }

  DiffDrive(uint8_t AA, uint8_t AB, uint8_t BA, uint8_t BB, uint16_t motorConst, real_t wheelBase)
		:Locomotion(),
		 DiffDriveMixin(),
		 SimpleHBridge(AA, AB, BA, BB, motorConst)
  {
    _wheelBase = wheelBase;
		//thrust_off_timeout = 0;
  }

  virtual void begin()
  {
		Locomotion::begin();
		SimpleHBridge::begin();
	}

	virtual void setThrust(const Quaternion& thrust) {
		Locomotion::setThrust(thrust);
		setCurrentThrust(thrust);
		setDiffSpeed(thrust.x, thrust.w);
	}

protected:
	virtual void setSpeed(real_t a, real_t b)
	{
		setLeftSpeed(a);
		setRightSpeed(b);
	}
};
};

#endif
