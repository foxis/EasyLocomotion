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
#include "hal/SimpleHBridge.h"
#include "controllers/differentialdrive.h"

namespace Locomotion {

template<typename T>
class _DiffDrive : public _Locomotion<T>, protected _DifferentialDriveController<T>, protected SimpleHBridge
{
public:
  _DiffDrive(uint8_t AA, uint8_t AB, uint8_t BA, uint8_t BB, T wheelBase)
    :_DiffDrive(AA, AB, BA, BB, PWMRANGE+1, wheelBase)
  {

  }

  _DiffDrive(uint8_t AA, uint8_t AB, uint8_t BA, uint8_t BB, uint16_t motorConst, T wheelBase)
		:_Locomotion<T>(),
		 _DifferentialDriveController<T>(),
		 SimpleHBridge(AA, AB, BA, BB, motorConst)
  {
    _wheelBase = wheelBase;
		//thrust_off_timeout = 0;
  }

  virtual void begin()
  {
		_Locomotion<T>::begin();
		_SimpleHBridge::begin();
	}

	virtual void setThrust(const _Quaternion<T>& thrust) {
		_Locomotion<T>::setThrust(thrust);
		setCurrentThrust(thrust);
		setDiffSpeed(thrust.x, thrust.w);
	}

protected:
	virtual void setSpeed(T a, T b)
	{
		setLeftSpeed(a);
		setRightSpeed(b);
	}
};

typedef _DiffDrive<real_t> DiffDrive;

};

#endif
