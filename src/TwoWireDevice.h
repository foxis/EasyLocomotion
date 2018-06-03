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

#if !defined(TWOWIREDEVICE_H)
#define TWOWIREDEVICE_H

#include <Arduino.h>
#include <Wire.h>
#include "readwritemixin.h"

class TwoWireDevice : public ReadWriteMixin {
protected:
	uint8_t addr;
	TwoWire * wire;
public:
	TwoWireDevice(TwoWire * wire, uint8_t addr) {
		this->addr = addr;
		this->wire = wire;
	}

	virtual void begin(bool init) {
		if (init)
			wire->begin();
	}

	virtual void read(uint8_t reg, uint8_t * out, uint8_t max_len) {
		if (reg != 0xFF)
			wire->write(reg);
		wire->requestFrom(addr, max_len);
    while (max_len--)
      *(out++) = wire->read();
  }

  virtual void write(uint8_t reg, const uint8_t * data, size_t len) {
      wire->beginTransmission(addr);
			wire->write(reg);
      if (data != NULL) wire->write(data, len);
      wire->endTransmission();
  }


};

#endif
