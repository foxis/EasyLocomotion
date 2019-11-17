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

#if !defined(HBRIDGE_TB724_H)
#define HBRIDGE_TB724_H

#include "MotorDriverBase.h"

namespace Locomotion {

class HBridge_TB724 : public MotorDriverBase {
	uint8_t _AA, _AB, _BB, _BA;
	uint8_t _PWMA, _PWMB;
	uint8_t _EN;
	uint16_t _motorConst;

public:
	HBridge_TB724(uint8_t AA, uint8_t AB, uint8_t PWMA,
								uint8_t BA, uint8_t BB, uint8_t PWMB,
								uint8_t EN, uint16_t motorConst) {
		_AA = AA;
		_BB = BB;
		_AB = AB;
		_BA = BA;
		_EN = EN;
		_PWMA = PWMA;
		_PWMB = PWMB;
		_motorConst = motorConst;
	}

	virtual void begin()
	{
		pinMode(_AA, OUTPUT);
		pinMode(_AB, OUTPUT);
		pinMode(_BA, OUTPUT);
		pinMode(_BB, OUTPUT);
		pinMode(_EN, OUTPUT);
		pinMode(_PWMA, OUTPUT);
		pinMode(_PWMB, OUTPUT);
		enable(false);
		setLeftSpeed(0);
		setRightSpeed(0);
	}

	virtual void enable(bool en)
	{
		digitalWrite(_EN, en);
	}

	virtual void setMotorsSpeed(real_t left, real_t right)
	{
		setLeftSpeed(left);
		setRightSpeed(right);
	}

	virtual void setLeftSpeed(real_t a)
	{
		setMotorSpeed(_AA, _AB, _PWMA, a);
	}
	virtual void setRightSpeed(real_t b)
	{
		setMotorSpeed(_BA, _BB, _PWMB, b);
	}

private:
	void setMotorSpeed(uint8_t A, uint8_t B, uint8_t PWM, real_t a)
	{
		uint8_t dirA;
		uint8_t dirB;

		if (a == 0.0) {
			dirA = dirB = HIGH;
		} else if (a > 0.0 && a <= 1.0) {
			dirA = HIGH;
			dirB = LOW;
		} else if (a > 1.0) {
			dirA = dirB = LOW;
		} else {
			dirA = LOW;
			dirB = HIGH;
		}

		uint16_t motor = min((uint16_t)_motorConst, (uint16_t)abs(a * _motorConst));
		digitalWrite(A, dirA);
		digitalWrite(B, dirB);
		analogWrite(PWM, motor);
	}
};

}
#endif // HBRIDGE_TB724_H
