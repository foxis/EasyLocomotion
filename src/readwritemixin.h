#if !defined(READWRITEMIXIN_H)
#define READWRITEMIXIN_H

#include <Arduino.h>

class ReadWriteMixin {
public:

	virtual void read(uint8_t reg, uint8_t * out, uint8_t max_len) = 0;
	virtual void write(uint8_t reg, const uint8_t * data, size_t len) = 0;

	uint8_t read8(uint8_t reg) {
    uint8_t data[1] = {reg};
    read(reg, data, sizeof(data));
    return data[0];
  }

	uint16_t read16(uint8_t reg) {
    uint8_t data[2] = {};
    read(reg, data, sizeof(data));
    return data[0] | (data[1] << 8);
  }

	void write8(uint8_t reg, uint8_t d) {
		uint8_t data[] = {d};
		write(reg, data, sizeof(data));
	}

	void write8(uint8_t d) {
		write(d, NULL, 0);
	}

	void write16(uint8_t reg, uint16_t d) {
		uint8_t data[] = {(uint8_t)d, (uint8_t)(d>>8)};
		write(reg, data, sizeof(data));
	}
	void write32(uint8_t reg, uint32_t d) {
		uint8_t data[] = {(uint8_t)d, (uint8_t)(d>>8), (uint8_t)(d>>16), (uint8_t)(d>>24)};
		write(reg, data, sizeof(data));
	}
};

#endif
