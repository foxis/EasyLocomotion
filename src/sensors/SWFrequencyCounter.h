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

class SoftwareFrequencyCounter {
	unsigned long interval;
	volatile unsigned long last_now;
	volatile unsigned long last_tick;
	volatile uint32_t counter;
	volatile real_t frequency;

public:
	SoftwareFrequencyCounter(unsigned long interval_ms) {
		interval = interval_ms * 1000;
		counter = 0;
		frequency = 0;
		last_now = micros();
	}

	real_t lastFrequency(unsigned long now) {
		return frequency;
	}

	void update(unsigned long now) {
		register unsigned long delta;
		if (now <= last_tick) return;
		//noInterrupts();
		delta = calc_delta(last_tick, now);
		if (delta > interval) {
			last_tick = now;
			calc_frequency(counter, now, interval);
		}
		//interrupts();
	}

	void count(unsigned long now) {
		last_tick = now;
		calc_frequency(++counter, now, interval);
	}

private:
	inline unsigned long calc_delta(register unsigned long lnow, register unsigned long now) {
		if (now >= lnow) {
			return now - lnow;
		} else {
			return now + (ULONG_MAX - lnow);
		}
	}

	inline void calc_frequency(uint32_t cnt, unsigned long now, unsigned long _interval) {
		register unsigned long delta = calc_delta(last_now, now);
		if (delta > _interval) {
			last_now = now;
			counter = 0;
			frequency = ((cnt * 1000.0) / (real_t)delta) * 1000.0;
		}
	}
};

}
#endif // SWFREQUENCYCOUNTER_H
