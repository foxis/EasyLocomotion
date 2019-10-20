#if !defined(HEXAPODDRIVE_H)
#define HEXAPODDRIVE_H

#include <Arduino.h>
#include "Locomotion.h"
#include "effectors/PCA9685.h"
#include "math_utils.h"
#include <vector>
#include <map>
#include <string>

namespace Locomotion {

typedef struct servoConfig_struct {
	uint8_t expander_addr : 4;				// expander i2c address where this motor is connected to
	uint8_t addr : 4;						// motor id on the expander

	// Servo motor position settings
	// Servo signal is calculated using this formula:
	// servo_value = x * k + c, where: 
	//		x - calculated segment position
	//		k - servo motor constant
	//		b - servo motor bias
	int16_t k;
	int16_t b;
} servoConfig_t;

template<typename T, typename KINEMATICS_T, size_t DOF> class Limb {
protected:
	KINEMATICS_T & kinematics;
	const _Vector3D<T> origin;
	const _Vector3D<T> orientation;
	T _Vector3D target;

public:
	T current_angles[DOF];
	T target_angles[DOF];
	T _Vector3D<T> position;

public:
	Limb(KINEMATICS_T & kinematics, const _Vector3D<T> & origin, const _Vector3D<T> & orientation,
		const _Vector3D<T> & position) 
		: kinematics(kinematics), origin(origin), orientation(orientation) {
		for (size_t i = 0; i < DOF; i ++) {
			this->current_angles[i] = this->kinematics.config[i].constraints.middle();
		}
	}

	virtual void begin() {
		this->kinematics.inverse(this->position, this->current_angles, this->target_angles, 0.1, 10);
	}

	virtual void loop(unsigned long now, unsigned long last_now, std::map<uint8_t, PCA9685>& expanders) {
		// TODO iterate over segments
	}

	virtual _Vector3D<T> set_target(const _Vector3D<T> & target) {
		_Vector3D<T> tmp(target);
		// TODO: transform body coordinates to leg coordinates
		tmp -= this->origin;
	}

	virtual _Vector3D<T> get_position() const {
		_Vector3D<T> tmp(this->position);
		// TODO: transform leg coordinates to body coordinates
		return tmp + this->origin;
	}
};

/// Gait is defined as a series of end effector positions
/// encoded by 0a-zA-Z as 0-64 in X, Y and Z. e.g. "000zZ0" as 0x0x0, 32x64x0
class Gait {
	std::vector<std::string> stages;
	std::vector<Limb> * limbs;
	unsigned long last_now;

public:
	Gait() {}
	Gait(const char ** stages, size_t stage_count) {

	}

	virtual void begin(std::vector<Limb> * limbs) {
		// TODO iterate over expanders
		// TODO iterate over limbs
	}

	virtual void loop(unsigned long now) {
		// TODO iterate over limbs
		last_now = now;
	}
};

template<typename T, typename KINEMATICS_T, size_t DOF>
class HexapodDrive : public Locomotion {
protected:
		Limb<T, KINEMATICS_T, DOF> *limbs;

		std::map<uint8_t, PCA9685> expanders;
		std::map<std::string, Gait*> gaits;
		unsigned long last_now;

public:
	HexapodDrive(TwoWire * wire, servoConfig_t * motors)
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

	Gait* getGait(const std::string& name) {
		return gaits[name];
	}
	void addGait(const std::string& name, Gait* gait) {
		gaits.insert(std::pair<std::string, Gait*>(name, gait));
	}
};


} // namespace

#endif
