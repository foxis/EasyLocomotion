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

#if !defined(SIMPLEHBRIDGE_H)
#define SIMPLEHBRIDGE_H

#include <Arduino.h>
#include "../math_utils.h"

namespace Locomotion {

class SimpleHBridge {
	uint8_t _AA, _AB, _BB, _BA;
	uint16_t _motorConst;

public:
	SimpleHBridge(uint8_t AA, uint8_t AB, uint8_t BA, uint8_t BB, uint16_t motorConst) {
		_AA = AA;
		_BB = BB;
		_AB = AB;
		_BA = BA;
		_motorConst = motorConst;
	}

	virtual void begin()
	{
		pinMode(_AA, OUTPUT);
		pinMode(_AB, OUTPUT);
		pinMode(_BA, OUTPUT);
		pinMode(_BB, OUTPUT);
	}

	void setSpeed(real_t a, real_t b)
	{
		setLeftSpeed(a);
		setRightSpeed(b);
	}

	void setMotorSpeed(uint8_t A, uint8_t B, real_t a)
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

}
#endif // SIMPLEHBRIDGE_H