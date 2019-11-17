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

#if !defined(SWFREQUENCYCOUNTER_H)
#define SWFREQUENCYCOUNTER_H

#include <Arduino.h>
#include <limits.h>
#include "../math_utils.h"

namespace Locomotion {

template <class T>
class SoftwareFrequencyCounter {
	volatile unsigned long last_now;
	volatile T last_counter;
	volatile T counter;
	volatile real_t frequency;
	bool nointerrupts;

public:
	SoftwareFrequencyCounter(bool nointerrupts = false) {
		counter = 0;
		last_counter = 0;
		frequency = 0;
		this->nointerrupts = nointerrupts;
		last_now = micros();
	}

	real_t lastFrequency() {
		return frequency;
	}
	real_t lastCounter() {
		return last_counter;
	}

	void update(unsigned long now) {
		T cnt;
		if (nointerrupts) noInterrupts();
		cnt = counter;
		if (nointerrupts) interrupts();

		calc_frequency(cnt, now);
	}

	inline void count(register char add) {
		counter += add;
	}

private:
	inline unsigned long calc_delta(register unsigned long lnow, register unsigned long now) {
		if (now >= lnow) {
			return now - lnow;
		} else {
			return now + (ULONG_MAX - lnow);
		}
	}

	inline void calc_frequency(T cnt, unsigned long now) {
		unsigned long delta = calc_delta(last_now, now);
		frequency = (((cnt - last_counter) * 1000.0) / (real_t)delta) * 1000.0;
		last_counter = cnt;
		last_now = now;
	}
};

}
#endif // SWFREQUENCYCOUNTER_H
