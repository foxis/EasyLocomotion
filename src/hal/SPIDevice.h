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

#include <SPI.h>
#include "readwritemixin.h"

namespace Locomotion {

class SPIDevice : public ReadWriteMixin
{
protected:
	uint8_t _ss_pin;
	uint8_t _reset_pin;
	SPIClass * _spi;
	uint16_t reset_delay;
	uint16_t register_delay;
	uint16_t transfer_delay;

public:
	SPIDevice(uint8_t ss_pin, uint8_t reset) : SPIDevice(&SPI, ss_pin, reset) {}
	SPIDevice(SPIClass * spi, uint8_t ss_pin, uint8_t reset) {
		_ss_pin = ss_pin;
		_reset_pin = reset;
		_spi = spi;
		reset_delay = 35;
		register_delay = 75;
		transfer_delay = 10;
	}

	virtual void reset() {
		digitalWrite(_reset_pin, HIGH);
		delay(1); // reset pulse >10us
		digitalWrite(_reset_pin, LOW);
		delay(reset_delay); // 35ms from reset to functional
	}

	virtual void begin(bool init) {
		if (init)
			_spi->begin();

		pinMode(_ss_pin, OUTPUT);
		pinMode(_reset_pin, OUTPUT);
		digitalWrite(_ss_pin, HIGH);
		digitalWrite(_reset_pin, HIGH);

		reset();
	}

	virtual void read(uint8_t reg, uint8_t * p, size_t size)
	{
	  digitalWrite(_ss_pin, LOW);
	  _spi->transfer(reg);
	  delayMicroseconds(register_delay);
		while (size--) {
			*(p++) =  _spi->transfer(0xff);
			delayMicroseconds(transfer_delay);
		}
	  digitalWrite(_ss_pin, HIGH);
	  delayMicroseconds(5);
	}

	virtual void write(uint8_t reg, const uint8_t * p, size_t size)
	{
	  digitalWrite(_ss_pin, LOW);
	  _spi->transfer(reg);
	  delayMicroseconds(register_delay);
		while (size--) {
			_spi->transfer(*(p++));
			delayMicroseconds(transfer_delay);
		}
	  digitalWrite(_ss_pin, HIGH);
	  delayMicroseconds(5);
	}

};

}

#endif
