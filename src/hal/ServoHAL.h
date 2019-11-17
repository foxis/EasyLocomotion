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

#if !defined(SERVO_HAL_H)
#define SERVO_HAL_H

namespace Locomotion {

///
/// stores servo motor calibration parameters.
/// Servo control is calculated as:
/// y = x * k + b,
/// Where y is servo control variable
/// x - angle in radians
typedef struct _ServoConfig_struct {
	int16_t k;
	int16_t b;
} ServoConfig_t;

///
/// Servo controller HAL
/// T - angle type
/// T1 - servo motor control type
/// N - number of servo motors
template<typename T, typename T1, size_t N>
class _ServoHAL {
protected:
    const _ServoConfig_t * config;
    T1 positions[N];

public:
    ServoHAL(const _ServoConfig_t * config) : config(config)
    {
    }

    virtual void begin(bool init) = 0;
    virtual void end() = 0;

    virtual void set_pos(size_t i, T pos) {
        T k = config[i].k, b = config[i].b;
        positions[i] = (T1)(pos * k + b);
    }
    virtual real_t get_pos(size_t i) const {
        T k = config[i].k, b = config[i].b;
        return (positions[i] - b) / k;
    }
    virtual void send() = 0;
    virtual void receive() = 0;

    ///
    /// calibrates servo motor using two known positions
    ///
    virtual void calibrate(real_t x1, real_t x2, T1 y1, T1 y2, int16_t * k, int16_t * b) const {
        *k = (y1 - y2) / (x1 - x2);
        *b = y1 - *k * x1;
    }
};

}

#endif