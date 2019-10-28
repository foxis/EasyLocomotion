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

#if !defined(GENERAL_KINEMATICS_H)
#define GENERAL_KINEMATICS_H

#include "../locomotion.h"
#include "kinematics.h"

namespace Locomotion {

typedef enum {
    PLANAR = 0,
    LINEAR = 1,
} JointType_t;

template<typename T>
struct _GeneralJoint_t {
    JointType_t type;
    _ConstraintSegment<T> constraints;
    _Vector3<T> axis;
    _Vector3<T> segment;
};

template<typename T, size_t DOF> class _GeneralKinematics : public _LimbKinematicsModel<T> {
public:
    const _GeneralJoint_t<T> * config;
    const _ConstraintVolume<T>& working_space;

public:
    _PlanarKin_GeneralKinematicsematics(const _GeneralJoint_t<T> * joints, const _ConstraintVolume<T> & working_space)
        : config(joints), working_space(working_space) {
    }
    ~_GeneralKinematics() {
    }

    /// Performs forward kinematics 
    virtual bool forward(const T * joint_value_arr, _Vector3D<T> & effector) {
        _Matrix3x3 transform;
        _Matrix3x3 current_transform;        
        const _GeneralJoint_t<T> * cp = config;
        const T * jp = joint_value_arr;

        transform.eye();
        effector.fill(0);

        for (size_t i = 0; i < DOF; i++) {
            const T value = cp->constraints.limit(*ap);
            if (cp->type == PLANAR) {
                // TODO - calculate rotation matrix
                transform *= current_transform;
                effector += current_transform * cp->segment;
            } else if (cp->type == LINEAR) {
                effector += cp->axis * value;
            }
            ++cp;
            ++ap;
        }
        return true;
    }

    virtual bool inverse(const _Vector3D<T> & target, const T * current_angle_arr, T * angle_arr, _Vector3D<T> & actual, T eps, size_t max_iterations) {
        #if defined IK_SOLVER_CCD
        #elif IK_SOLVER_JACOBIAN
        #error Jacobian IK Solver not implemented
        #else
        #error Please specify a solver (IK_SOLVER_CCD or IK_SOLVER_JACOBIAN)
        #endif
    }

    virtual size_t dof() const { return DOF; }
    virtual _ConstraintVolume<T> target_space() const { return working_space; }
};

typedef _PlanarJoint_t<real_t> PlanarJoint_t;

};

#include "PlanarKinematics2DOF.h"
#include "PlanarKinematics3DOF.h"

#endif