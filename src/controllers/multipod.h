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

#if !defined(MULTIPOD_CONTROLLER_H)
#define MULTIPOD_CONTROLLER_H

#include "../kinematics/kinematics.h"
#include "../locomotion.h"

namespace Locomotion {

template<typename T, size_t LIMBS>
class _MultipodController {
protected:
    _BodyModel<T, LIMBS> * body;
    _Locomotion<T> * locomotion;

public:
	_MultipodController() {}

    void set_body(_BodyModel<T, LIMBS> * body) {
        this->body = body;
    }
    void set_locomotion(_Locomotion<T> * locomotion) {
        this->locomotion = locomotion;
    }

    virtual void begin() {}
    virtual void end() {}
    virtual void loop(timestamp_t now, timestamp_t last_now) {}
};

#include "_hexapod.h"

template<size_t LIMBS>
using MultipodController = _MultipodController<real_t, LIMBS>;

};

#endif
