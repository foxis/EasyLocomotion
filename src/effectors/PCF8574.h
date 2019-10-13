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

#include <Arduino.h>
#include "../TwoWireDevice.h"

namespace Locomotion {

//
// PCF8574 is a bidirectional Input/Output
//
class PCF8574 : public TwoWireDevice {
public:
	PCF8574(TwoWire * wire)
		: PCF8574(wire, 0x20) { }
  PCF8574(TwoWire * wire, byte io_addr)
		: TwoWireDevice(wire, io_addr) {
  }

	void set(uint8_t data) {
		write8(data);
	}
	uint8_t get() {
		return read8(0xFF);
	}

	virtual void begin(bool init) {
		TwoWireDevice::begin(init);
		if (init)
			io_data = get();
	}

	void setPin(int8_t id, bool value) {
    if (value == LOW)
    {
        io_data &= ~(1 << id);
    }
    else
    {
        io_data |= (1 << id);
    }
    set(io_data);
  }
  bool getPin(int16_t id) {
		io_data = get();
    return io_data & (~(1 << id));
  }
protected:
  byte io_data;
};

}

#endif
