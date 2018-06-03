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

class SPIDevice : public ReadWriteMixin
{
	int _ss_pin;
	int _reset_pin;

public:
	typedef struct MD
	{
	  byte motion;
	  char dx, dy;
	  byte squal;
	  word shutter;
	  byte max_pix;
	} MD_t;

public:
	ADNS3080(int ss_pin, int reset) {
		_ss_pin = ss_pin;
		_reset_pin = reset;
		_init = false;
	}

	void reset() {
		digitalWrite(_reset_pin, HIGH);
		delay(1); // reset pulse >10us
		digitalWrite(_reset_pin, LOW);
		delay(35); // 35ms from reset to functional
	}

	void begin() {
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

	virtual void read(uint8_t reg, void * p, size_t size)
	{
	  digitalWrite(_ss_pin, LOW);
	  SPI.transfer(reg);
	  delayMicroseconds(75);
		while (size--) {
			*((uint8_t*)p++) =  SPI.transfer(0xff);
		}
	  digitalWrite(_ss_pin, HIGH);
	  delayMicroseconds(5);
	}

	virtual void write(uint8_t reg, void * p, size_t size)
	{
	  digitalWrite(_ss_pin, LOW);
	  SPI.transfer(reg);
	  delayMicroseconds(75);
		while (size--) {
			SPI.transfer(*((uint8_t*)p++));
		}
	  digitalWrite(_ss_pin, HIGH);
	  delayMicroseconds(5);
	}

};

#endif
