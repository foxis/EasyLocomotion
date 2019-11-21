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

#if !defined(PCA9685_H)
#define PCA9685_H

#include "../hal/TwoWireDevice.h"

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

namespace Locomotion {

//
// PCA9685 is a 16x PWM output controlled by i2c
//
class PCA9685 : public TwoWireDevice {
public:
	PCA9685(TwoWire * wire)
		: PCA9685(wire, 0x40 | 0b1010101) { }
  PCA9685(TwoWire * wire, uint8_t pwm_addr)
		: TwoWireDevice(wire, pwm_addr) {
  }

  virtual void begin(bool init) {
		TwoWireDevice::begin(init);
		if (init)
			reset();
  }

  void setFreq(float f) {
    byte prescale = floor(25000000 / (4096 * f) + 0.5) - 1;

    byte oldmode = read8(PCA9685_MODE1);
    byte newmode = (oldmode & 0x7F) | 0x10; // sleep

    write8(PCA9685_MODE1, newmode); // go to sleep
    write8(PCA9685_PRESCALE, prescale); // set the prescaler
    write8(PCA9685_MODE1, oldmode);
    delay(5);
    write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
    // This is why the beginTransmission below was not working.
    //  SerialUSB.print("Mode now 0x"); SerialUSB.println(read8(PCA9685_MODE1), HEX);
  }

  void setPWMraw(uint8_t id, uint16_t value, bool invert) {
    uint32_t on = 0;
    uint32_t off = invert ? 4095 - value : value;

    write32(LED0_ON_L + 4 * id, (off << 16) | on);
  }

  void setPWM(uint8_t id, float value, bool invert) {
    setPWMraw(id, value * 4095, invert);
  }

  void reset(void) {
    write8(PCA9685_MODE1, 0x0);
  }
};

}

#endif
