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

class DiffDrive : public Locomotion, public SimpleHBridge
{

  real_t _wheelBase;

  real_t _vTarget;
  real_t _wTarget;

public:
  DiffDrive(int AA, int AB, int BA, int BB, real_t wheelBase)
    :DiffDrive(AA, AB, BA, BB, PWMRANGE + 1, wheelBase)
  {

  }

  DiffDrive(int AA, int AB, int BA, int BB, int motorConst, real_t wheelBase)
		:Locomotion(),
		 SimpleHBridge(AA, AB, BA, BB, motorConst)
  {
    _wheelBase = wheelBase;
  }

  virtual void begin()
  {
		Locomotion::begin();
		SimpleHBridge::begin();
	}


	virtual void setThrust(const Quaternion& thrust) {
		Locomotion::setThrust(thrust);
		setCurrentThrust(thrust);
		setDiffSpeed(thrust.getMagnitude(), thrust.w);
		updated(millis());
	}


protected:
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
};

#endif
