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
 *
 * Code for ADNS3080 was copied and refactored from:
 *  https://github.com/Neumi/ESP8266_ADNS3080_reflectivity_finder
 *
 */

#if !defined(SPIDevice_H)
#define SPIDevice_H

#include <Arduino.h>
#include <SPI.h>
#include "readwritemixin.h"

namespace Locomotion {

class SPIDevice : public ReadWriteMixin
{
	uint8_t _ss_pin;
	uint8_t _reset_pin;

public:
	SPIDevice(uint8_t ss_pin, uint8_t reset) {
		_ss_pin = ss_pin;
		_reset_pin = reset;
	}

	virtual void reset() {
		digitalWrite(_reset_pin, HIGH);
		delay(1); // reset pulse >10us
		digitalWrite(_reset_pin, LOW);
		delay(35); // 35ms from reset to functional
	}

	virtual void begin() {
		SPI.begin();
		SPI.setClockDivider(SPI_CLOCK_DIV32);
	  SPI.setDataMode(SPI_MODE3);
	  SPI.setBitOrder(MSBFIRST);
		pinMode(_ss_pin, OUTPUT);
		pinMode(_reset_pin, OUTPUT);
		digitalWrite(_ss_pin, HIGH);
		digitalWrite(_reset_pin, HIGH);

		reset();
	}

	virtual void read(uint8_t reg, uint8_t * p, size_t size)
	{
	  digitalWrite(_ss_pin, LOW);
	  SPI.transfer(reg);
	  delayMicroseconds(75);
		while (size--) {
			*(p++) =  SPI.transfer(0xff);
		}
	  digitalWrite(_ss_pin, HIGH);
	  delayMicroseconds(5);
	}

	virtual void write(uint8_t reg, const uint8_t * p, size_t size)
	{
	  digitalWrite(_ss_pin, LOW);
	  SPI.transfer(reg);
	  delayMicroseconds(75);
		while (size--) {
			SPI.transfer(*(p++));
		}
	  digitalWrite(_ss_pin, HIGH);
	  delayMicroseconds(5);
	}

};

}

#endif
