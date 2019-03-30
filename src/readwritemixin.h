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

#if !defined(READWRITEMIXIN_H)
#define READWRITEMIXIN_H

#include <Arduino.h>

namespace Locomotion {

class ReadWriteMixin {
public:
	enum ByteOrder {MSBFirst, LSBFirst};
	typedef struct {
		uint8_t reg;
		union {
			uint8_t val8;
			uint16_t val16;
			uint32_t val32;
		} val;
		int bits;
	} calib_data_multiple_t;

	typedef struct {
		uint8_t reg;
		uint8_t val;
	} calib_data8_t;

	typedef struct {
		uint8_t reg;
		uint16_t val;
	} calib_data16_t;

	typedef struct {
		uint8_t reg;
		uint32_t val;
	} calib_data32_t;

	virtual void read(uint8_t reg, uint8_t * out, size_t max_len) = 0;
	virtual void write(uint8_t reg, const uint8_t * data, size_t len) = 0;

	uint8_t read8(uint8_t reg) {
    uint8_t data[1] = {reg};
    read(reg, data, sizeof(data));
    return data[0];
  }

	uint16_t read16(uint8_t reg, ByteOrder order = LSBFirst) {
    uint8_t data[2] = {};
    read(reg, data, sizeof(data));
		if (order == LSBFirst)
    	return (uint16_t)(data[0]) | ((uint16_t)(data[1]) << 8);
		else
			return (uint16_t)(data[1]) | ((uint16_t)(data[0]) << 8);
  }

	uint32_t read24(uint8_t reg, ByteOrder order = LSBFirst) {
    uint8_t data[3] = {};
    read(reg, data, sizeof(data));
		if (order == LSBFirst)
			return (uint32_t)(data[0]) | ((uint32_t)(data[1]) << 8) | ((uint32_t)(data[2]) << 16);
		else
			return (uint32_t)(data[2]) | ((uint32_t)(data[1]) << 8) | ((uint32_t)(data[0]) << 16);
  }

	uint32_t read32(uint8_t reg, ByteOrder order = LSBFirst) {
    uint8_t data[4] = {};
    read(reg, data, sizeof(data));
		if (order == LSBFirst)
			return (uint32_t)(data[0]) | ((uint32_t)(data[1]) << 8) | ((uint32_t)(data[2]) << 16) | ((uint32_t)(data[3]) << 24);
		else
			return (uint32_t)(data[3]) | ((uint32_t)(data[2]) << 8) | ((uint32_t)(data[1]) << 16) | ((uint32_t)(data[0]) << 24);
  }

	void write8(uint8_t reg, uint8_t d) {
		uint8_t data[] = {d};
		write(reg, data, sizeof(data));
	}

	void write8(uint8_t d) {
		write(d, NULL, 0);
	}

	void write16(uint8_t reg, uint16_t d, ByteOrder order = LSBFirst) {
		if (order == LSBFirst) {
			uint8_t data[] = {(uint8_t)d, (uint8_t)(d >> 8)};
			write(reg, data, sizeof(data));
		}
		else {
			uint8_t data[] = {(uint8_t)(d >> 8), (uint8_t)d};
			write(reg, data, sizeof(data));
		}
	}

	void write24(uint8_t reg, uint32_t d, ByteOrder order = LSBFirst) {
		if (order == LSBFirst) {
			uint8_t data[] = {(uint8_t)d, (uint8_t)(d >> 8), (uint8_t)(d >> 16)};
			write(reg, data, sizeof(data));
		}
		else {
			uint8_t data[] = {(uint8_t)(d >> 16), (uint8_t)(d >> 8), (uint8_t)d};
			write(reg, data, sizeof(data));
		}
	}

	void write32(uint8_t reg, uint32_t d, ByteOrder order = LSBFirst) {
		if (order == LSBFirst) {
			uint8_t data[] = {(uint8_t)d, (uint8_t)(d >> 8), (uint8_t)(d >> 16), (uint8_t)(d >> 24)};
			write(reg, data, sizeof(data));
		}
		else {
			uint8_t data[] = {(uint8_t)(d >> 24), (uint8_t)(d >> 16), (uint8_t)(d >> 8), (uint8_t)d};
			write(reg, data, sizeof(data));
		}
	}

	void write_bulk(const calib_data8_t * data, size_t cnt) {
		while (cnt) {
			write8(data->reg, data->val);
			++data;
			--cnt;
		}
	}

	void write_bulk(const calib_data16_t * data, size_t cnt, ByteOrder order = LSBFirst) {
		while (cnt) {
			write16(data->reg, data->val, order);
			++data;
			--cnt;
		}
	}

	void write_bulk(const calib_data32_t * data, size_t cnt, ByteOrder order = LSBFirst) {
		while (cnt) {
			write32(data->reg, data->val, order);
			++data;
			--cnt;
		}
	}

	void write_bulk(const calib_data_multiple_t * data, size_t cnt, ByteOrder order = LSBFirst) {
		while (cnt) {
			switch (data->bits) {
				case 8: write8(data->reg, data->val.val8); break;
				case 16: write16(data->reg, data->val.val16, order); break;
				case 24: write24(data->reg, data->val.val32, order); break;
				case 32: write32(data->reg, data->val.val32, order); break;
				default:
					return;
			}
			++data;
			--cnt;
		}
	}

	void write_bulk_P(const calib_data8_t * data, size_t cnt) {
		calib_data8_t tmp;
		while (cnt) {
			memcpy_P(&tmp, data, sizeof(tmp));
			write8(data->reg, tmp.val);
			++data;
			--cnt;
		}
	}

	void write_bulk_P(const calib_data16_t * data, size_t cnt, ByteOrder order = LSBFirst) {
		calib_data16_t tmp;
		while (cnt) {
			memcpy_P(&tmp, data, sizeof(tmp));
			write16(data->reg, data->val, order);
			++data;
			--cnt;
		}
	}

	void write_bulk_P(const calib_data32_t * data, size_t cnt, ByteOrder order = LSBFirst) {
		calib_data32_t tmp;
		while (cnt) {
			memcpy_P(&tmp, data, sizeof(tmp));
			write32(data->reg, tmp.val, order);
			++data;
			--cnt;
		}
	}

	void write_bulk_P(const calib_data_multiple_t * data, size_t cnt, ByteOrder order = LSBFirst) {
		calib_data_multiple_t tmp;
		while (cnt) {
			memcpy_P(&tmp, data, sizeof(tmp));
			switch (tmp.bits) {
				case 8: write8(data->reg, tmp.val.val8); break;
				case 16: write16(data->reg, tmp.val.val16, order); break;
				case 24: write24(data->reg, tmp.val.val32, order); break;
				case 32: write32(data->reg, tmp.val.val32, order); break;
				default:
					return;
			}
			++data;
			--cnt;
		}
	}

	bool wait_while(uint8_t reg, uint8_t mask, uint8_t result, unsigned long timeout_us) {
		unsigned long now = micros();
	  while ((read8(reg) & mask) == result)
	  {
	    if (timeout_us > 0 && (micros() - now > timeout_us))
	      return false;
	  }
		return true;
	}

	bool wait_until(uint8_t reg, uint8_t mask, uint8_t result, unsigned long timeout_us) {
		unsigned long now = micros();
	  while ((read8(reg) & mask) != result)
	  {
			if (timeout_us > 0 && (micros() - now > timeout_us))
	      return false;
	  }
		return true;
	}
};

}

#endif
