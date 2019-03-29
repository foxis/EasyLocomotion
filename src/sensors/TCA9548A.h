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

#if !defined(TCA9548A_H)
#define TCA9548A_H

#include <Wire.h>
#include "../TwoWireDevice.h"

namespace Locomotion {

#define TCA9548A_BASE_ADDRESS 0x70

//
// TCA9548A is a 8x I2C mux
//
class TCA9548A : public TwoWireDevice {
public:
	TCA9548A(TwoWire * wire)
		: TCA9548A(wire, TCA9548A_BASE_ADDRESS) { }
  TCA9548A(TwoWire * wire, byte address_offset)
		: TwoWireDevice(wire, TCA9548A_BASE_ADDRESS + address_offset) {
  }

  virtual void begin(bool init) {
		TwoWireDevice::begin(init);
		if (init)
			reset();
  }

  void setChannel(byte num) {
		setChannelMask(_BV(num % 8));
  }

	void setChannelMask(byte mask) {
		 write8(mask);
	}

  void reset(void) {
    write8(0x00);
  }
};

}

#endif
