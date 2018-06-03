/*  ESPBalancingRobot
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

#include "Arduino.h"

#include "locomotion.h"

namespace Locomotion {

class DiffDrive : public Locomotion
{
  int _AA, _AB, _BB, _BA;
  int _motorConst;
  real_t _wheelBase;

  real_t _vTarget;
  real_t _wTarget;

public:
  DiffDrive(int AA, int AB, int BA, int BB, real_t wheelBase)
    :DiffDrive(AA, AB, BA, BB, PWMRANGE + 1, wheelBase)
  {

  }

  DiffDrive(int AA, int AB, int BA, int BB, int motorConst, real_t wheelBase)
		:Locomotion()
  {
    _AA = AA;
    _BB = BB;
    _AB = AB;
    _BA = BA;
    _motorConst = motorConst;
    _wheelBase = wheelBase;
  }

  virtual void begin()
  {
    pinMode(_AA, OUTPUT);
    pinMode(_AB, OUTPUT);
    pinMode(_BA, OUTPUT);
    pinMode(_BB, OUTPUT);

		Locomotion::begin();
  }


	virtual void setThrust(const Quaternion& thrust) {
		Locomotion::setThrust(thrust);
		setCurrentThrust(thrust);
		setDiffSpeed(thrust.getMagnitude(), thrust.w);
		updated(millis());
	}


private:
	/// power - differential drive train power
	/// w - rotational power (normalized wL)
	/// all control parameters range -1.0 ... +1.0
	void setDiffSpeed(real_t power, real_t w)
	{
		real_t a = (2.0 * power + w) / 2.0;
		real_t b = (2.0 * power - w) / 2.0;
		setSpeed(a, b);
	}

  void setSpeed(real_t a, real_t b)
  {
    setLeftSpeed(a);
    setRightSpeed(b);
  }

  void setMotorSpeed(int A, int B, real_t a)
  {
    uint16_t motor = min((uint16_t)_motorConst, (uint16_t)abs(a * _motorConst));
    analogWrite(A, a > 0 ? motor : 0);
    analogWrite(B, a > 0 ? 0: motor);
  }

  void setLeftSpeed(real_t a)
  {
    setMotorSpeed(_AA, _AB, a);
  }
  void setRightSpeed(real_t b)
  {
    setMotorSpeed(_BA, _BB, -b);
  }
};
};
