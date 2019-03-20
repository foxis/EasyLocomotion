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

#if !defined(BATTLEDROIDSHIELD_H)
#define BATTLEDROIDSHIELD_H

#include <Arduino.h>
#include <Wire.h>
#include "PCA9685.h"
#include "MotorDriverBase.h"

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

class BattleDroidShield : public MotorDriverBase {
	TwoWireDevice io;
	PCA9685 pwm;
public:
	BattleDroidShield(TwoWire * wire)
		:BattleDroidShield(wire, 0x40 | 0b1010101, 0x20)
	{}
  BattleDroidShield(TwoWire * wire, byte pwm_addr, byte io_addr)
		: io(wire, io_addr),
		  pwm(wire, pwm_addr) {
	}

	virtual void begin() {
		begin(true);
	}

  virtual void begin(bool init) {
    io.begin(false);
		pwm.begin(false);
		if (init)
			wire->begin();
    reset();
  }

	virtual void enable(bool) {}

  virtual void setMotorsSpeed(real_t a, real_t b) {
    // 0 STBY, 1 !OE, 2/3 A1/A2, 4/5 B1/B2
    uint8_t dirA = a > 0 ? 0x10 : a == 0 ? 0x11 : a > 1 ? 0x00 : 0x01;
    uint8_t dirB = b > 0 ? 0x10 : b == 0 ? 0x11 : b > 1 ? 0x00 : 0x01;
    io_data = (io_data & 0x03) | dirA << 2 | dirB << 4;
    io.write8(io_data);
    pwm.setPWM(0, 4095 * abs(a));
    pwm.setPWM(1, 4095 * abs(b));
  }

	virtual void setLeftSpeed(real_t a) {}
	virtual void setRightSpeed(real_t b) {}

  bool IO(int16_t id, bool value) {
    if (value == LOW)
    {
        io_data &= ~(1 << id);
    }
    else
    {
        io_data |= (1 << id);
    }
    io.write8(io_data);
  }
  bool IO(int16_t id) {
    io.read(&io_data, sizeof(io_data));
    return io_data & (~(1 << id));
  }

  void reset(void) {
    io.write8(0x01); // enable PCA9685 and motor driver
    io.read(&io_data, sizeof(io_data));
    pwm.reset();
  }
protected:
  byte io_data;
};

}

#endif
