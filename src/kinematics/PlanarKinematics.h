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

#if !defined(PLANAR_KINEMATICS_H)
#define PLANAR_KINEMATICS_H

#include "../locomotion.h"
#include "kinematics.h"

namespace Locomotion {

typedef struct PlanarJoint_struct {
    BoundingSegment constraints;
    real_t length;
} PlanarJoint_t;

template<typename T, size_t DOF> class PlanarKinematics : public KinematicsModel<T> {
private:
    const PlanarJoint_t * config;

public:
    PlanarKinematics(const PlanarJoint_t * joints)
        : config(joints) {
    }
    ~PlanarKinematics() {
    }

    /// Performs planar forward kinematics 
    /// assuming that the first joint is rotation about y axis
    ///
    virtual _Vector3D<T> direct(const T * angle_arr) {
        _Vector3D<T> effector(config[0].length, 0, 0);
        real_t a = 0;
        for (size_t i = 1; i < DOF; i++) {
            a += angle_arr[i];
            real_t x = config[i].length * cos(a);
            real_t y = config[i].length * sin(a);
            effector += _Vector3D<T>(x, y, 0);
        }
        return _Vector3D<T>(effector.x * cos(angle_arr[0]), effector.y, effector.x * sin(angle_arr[0]));
    }

    virtual T inverse(const _Vector3D<T> & target, const T * current_angle_arr, T * angle_arr, T eps, size_t max_iterations) {
        #if defined IK_SOLVER_CCD
        #elif IK_SOLVER_JACOBIAN
        #error Jacobian IK Solver not implemented
        #else
        #error Please specify a solver (IK_SOLVER_CCD or IK_SOLVER_JACOBIAN)
        #endif
    }

};

};

#include "PlanarKinematics2DOF.h"
#include "PlanarKinematics3DOF.h"

#endif