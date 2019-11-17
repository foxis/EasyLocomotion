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

#if !defined(LOOPER_H)
#define LOOPER_H

#include "math_utils.h"

namespace Locomotion {

class Looper {
protected:
	timestamp_t last_now;

public:
	Looper() {}

	virtual void begin() = 0;

	virtual void loop(timestamp_t now) {
		last_now = now;
	}

	virtual void loop(timestamp_t now, timestamp_t last_now) {
		this->last_now = last_now;
		this->loop(now);
	}
};

}

#endif
