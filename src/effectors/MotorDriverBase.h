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

#if !defined(MOTORDRIVERBASE_H)
#define MOTORDRIVERBASE_H

#include <Arduino.h>
#include "../math_utils.h"

namespace Locomotion {

class MotorDriverBase {
public:
	virtual void begin() = 0;
	virtual void enable(bool en) = 0;

	virtual void setMotorsSpeed(real_t left, real_t right) = 0;
	virtual void setLeftSpeed(real_t a) = 0;
	virtual void setRightSpeed(real_t b) = 0;
};

}
#endif // MOTORDRIVERBASE_H
