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

#if !defined(ADNS3080_H)
#define ADNS3080_H

#include <Arduino.h>
#include "../SPIDevice.h"


#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17

namespace Locomotion {

class ADNS3080 : public SPIDevice
{
	bool _init;

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
	ADNS3080(uint8_t ss_pin, uint8_t reset) : SPIDevice(ss_pin, reset) {
		_init = false;
	}

	virtual void reset() {
		SPIDevice::reset();
		write8(ADNS3080_CONFIGURATION_BITS, 0x19);
	}

	virtual void begin() {
		SPIDevice::begin();

		uint8_t pid = read8(ADNS3080_PRODUCT_ID);
		if (pid != ADNS3080_PRODUCT_ID_VAL) {
			_init = false;
			return ;
		}

		// turn on sensitive mode
		write8(ADNS3080_CONFIGURATION_BITS, 0x19);
		_init = true;
	}

	void read_motion(MD_t * motion_data)
	{
		if (!_init) return;

		read(ADNS3080_MOTION_BURST, (uint8_t*)motion_data, sizeof(MD_t));

		motion_data->shutter = (motion_data->shutter << 8) | (motion_data->shutter >> 8);
	}

	// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
	// you must call mousecam_reset() after this if you want to go back to normal operation
	size_t frame_capture(uint8_t *pdata)
	{
		if (!_init) return 0;

	  write8(ADNS3080_FRAME_CAPTURE, 0x83);

		read(ADNS3080_PIXEL_BURST, pdata, ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y);
	  delayMicroseconds(14);

	  return ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y;
	}
};

}

#endif
