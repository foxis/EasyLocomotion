#if !defined(HEXAPODDRIVE_H)
#define HEXAPODDRIVE_H

#include <Arduino.h>
#include "Locomotion.h"
#include "effectors/PCA9685.h"
#include <vector>
#include <map>

namespace Locomotion {

typedef struct motorConfig_struct {
	uint8_t limb_id;							//
	uint8_t segment_id;						//
	uint8_t expander_addr;				// expander i2c address where this motor is connected to
	uint8_t addr;								  // motor id on the expander

	// Servo motor position settings
	uint16_t min;
	uint16_t max;
	uint16_t initial;
} motorConfig_t;

class Segment {
protected:
	motorConfig_t config;
	// positions range from 0 to 1
	// these will be mapped to servo motor positions accordingly based on min/max
public:
	real_t start;
	real_t current;
	real_t end;
	real_t speed;

public:
	Segment(const motorConfig_t * config) {
		this->config = *config;
		start = current = end = config->initial;
		speed = 0;
	}

	virtual void begin() {
		// TODO setup segment
	}
	virtual void loop(unsigned long now, unsigned long last_now, std::map<uint8_t, PCA9685>& expanders) {
		// TODO send segment info to effector
	}
};

class Limb {
protected:
	std::vector<Segment> segments;

public:
	Limb(const motorConfig_t * configs, size_t count) {
		// TODO setup segments
	}

	virtual void begin() {
		// TODO iterate over segments
	}

	virtual void loop(unsigned long now, unsigned long last_now, std::map<uint8_t, PCA9685>& expanders) {
		// TODO iterate over segments
	}

	virtual Segment& getSegment(size_t index) {
		return segments[index];
	}
	virtual size_t getSegmentCount() {
		return segments.size();
	}
};

class HexapodDrive : public Locomotion {
protected:
		std::vector<Limb> limbs;
		std::map<uint8_t, PCA9685> expanders;
		unsigned long last_now;

public:
	HexapodDrive(TwoWire * wire, motorConfig_t * motors, size_t motorNum)
		: Locomotion() {
			// TODO setup limbs and expanders
	}

	virtual void begin() {
		// TODO iterate over expanders
		// TODO iterate over limbs
	}

	virtual void loop(unsigned long now) {
		// TODO iterate over limbs
		last_now = now;
	}

	virtual Limb& getLimb(size_t index) {
		return limbs[index];
	}
	virtual size_t getLimbCount() {
		return limbs.size();
	}
};


} // namespace

#endif