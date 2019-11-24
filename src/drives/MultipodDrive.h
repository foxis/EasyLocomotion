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

#if !defined(MULTIPODDRIVE_H)
#define MULTIPODDRIVE_H

#include <Arduino.h>
#include "Locomotion.h"
#include "kinematics/kinematics.h"
#include "hal/ServoHAL.h"
#include "controllers/multipod.h"

namespace Locomotion {

template<typename T, size_t LIMBS, size_t SERVO_COUNT, typename T1 = uint16_t>
class _MultipodDrive : public _BodyModel<T, LIMBS>, public Locomotion {
protected:
    typedef _LimbKinematicsModel<T> * LimbModelPtr_t;

	_ServoHAL<T, T1, SERVO_COUNT> &hal;
	_MultipodController<T, LIMBS> &controller;

public:
	_MultipodDrive(_ServoHAL<T, T1, SERVO_COUNT> & hal, _MultipodController<T, LIMBS> & controller, 
				   LimbModelPtr_t * limbs, const _LimbConfig_t<T> * limb_config, 
				   _ConstraintVolume<T> & working_space, const _Vector3D<T> & position, const _Vector3D<T> & orientation)
		: _BodyModel<T, LIMBS>(limbs, limb_config, working_space, position, orientation),
		  _Locomotion<T>(), hal(hal), controller(controller) 
	{
		controller.set_body(this);
		controller.set_locomotion(this);
	}

	virtual void begin(timestamp_t now, bool init) {
		hal.begin(init);
		controller.begin();
		_Locomotion<T>::begin(now);
	}

	virtual void loop(timestamp_t now) {
		controller.loop(now, this->last_now);
		hal.set_pos(this->current_joints);
		hal.send();
		_Locomotion<T>::loop(now);
	}
};

template<size_t LIMBS, size_t SERVO_COUNT>
using MultipodDrive = _MultipodDrive<real_t, LIMBS, SERVO_COUNT>;

} // namespace

#endif
