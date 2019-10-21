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

#if !defined(KINEMATICS_H)
#define KINEMATICS_H

#include "locomotion.h"

namespace Locomotion {

#if !defined(IK_SOLVER_CCD) && !defined(IK_SOLVER_JACOBIAN)
#define IK_SOLVER_CCD
#endif

template<typename T> class _KinematicsModel {
public:
    virtual _Vector3D<T> direct(const T * angle_arr, _Vector3D<T> dst) {
        _Vector3D<T> tmp;
        direct(angle_arr, &tmp);
        return tmp;
    }
    virtual bool direct(const T * angle_arr, _Vector3D<T> dst) = 0;
    virtual T inverse(const _Vector3D<T> & target, const T * current_angle_arr, T * angle_arr, T eps, size_t max_iterations) = 0;
};

};

#endif