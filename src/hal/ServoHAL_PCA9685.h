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

#if !defined(SERVO_HAL_PCA9685_H)
#define SERVO_HAL_PCA9685_H

#include "ServoHAL.h"
#include "../effectors/PCA9685.h"

namespace Locomotion {

///
/// Servo motor HAL using PWM expansion chip PCA9685
/// Note: config array must be of size N
///       pca array must be the size of number of PCA9685 chips to
///       accomodate all servo motors
template<typename T, size_t N>
class _ServoHAL_PCA9685 : public _ServoHAL<T, uint16_t, N> {
    PCA9685 * pca9685;
    size_t num_boards;
    float frequency;

public:
    ServoHAL(const ServoConfig_t * config, PCA9685 * pca, float frequency) 
        : _ServoHAL<T, int16_t, N>(config), pca9685(pca), frequency(frequency)
    {
        num_boards = N >> 4;
        if (N & 0x0F != 0)
            num_boards++;
    }

    virtual void begin(bool init) {
        for (size_t i = 0; i < num_boards; i++) {
            pca9685[i].begin(init);
            pca9685[i].reset();
            pca9685[i].setFreq(frequency);
        }
    }
    virtual void end() {

    }

    virtual void send() {
        size_t M = N;
        int16_t *p = this->positions;
        for (size_t i = 0; i < num_boards; i++) {
            uint8_t K = MIN(M, 16);
            for (uint8_t j = 0; j < K; j++) {
                pca9865[i].setPWMraw(j, *p, false);
                ++p;
            }
            M -= K;
        }
    }
    virtual void receive() {

    }
};

template <size_t N>
using ServoHAL_PCA9685 = _ServoHAL_PCA9685<real_t, N>;

}

#endif