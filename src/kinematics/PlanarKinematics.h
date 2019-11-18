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

template<typename T>
struct _PlanarJoint_t {
    _ConstraintSegment<T> constraints;
    T length;
};

template<typename T, size_t DOF> class _PlanarKinematics : public _LimbKinematicsModel<T> {
public:
    const _PlanarJoint_t<T> * config;
    const _ConstraintVolume<T>& _working_space;

public:
    _PlanarKinematics(const _PlanarJoint_t<T> * joints, const _ConstraintVolume<T> & working_space)
        : config(joints), _working_space(working_space) {
    }
    ~_PlanarKinematics() {
    }

    /// Performs planar forward kinematics 
    /// assuming that the first joint is rotation about y axis
    ///
    virtual bool forward(const T * param_arr, _Vector<T, 3> & dst) {
        const T * ap = param_arr + 1;
        const _PlanarJoint_t<T> *cp = this->config + 1;
        _Vector3D<T> effector(config[0].length, 0, 0);
        T a = 0;

        for (size_t i = 1; i < DOF; i++) {
            a += cp->constraints.limit(*ap);

            real_t x = cp->length * cos(a);
            real_t z = cp->length * sin(a);
            effector += _Vector3D<T>(x, 0, z);

            ++cp;
            ++ap;
        }
        const T tmp_x = effector.x;
        dst.data()[0] = tmp_x * cos(param_arr[0]);
        dst.data()[1] = tmp_x * sin(param_arr[0]);
        dst.data()[2] = effector.z;
        return true;
    }

    virtual bool inverse(const _Vector<T, 3> & target, const T * current_param_arr, T * param_arr, _Vector<T, 3> & actual, T eps, size_t max_iterations) {
        #if defined(IK_SOLVER_CCD)
        #elif defined(IK_SOLVER_JACOBIAN)
        #else
        #error Please specify a solver (IK_SOLVER_CCD or IK_SOLVER_JACOBIAN)
        #endif
        return false;
    }

    virtual size_t dof() const { return DOF; }
    virtual _ConstraintVolume<T> working_space() const { return _working_space; }
};

typedef _PlanarJoint_t<real_t> PlanarJoint_t;

};

#include "_PlanarKinematics2DOF.h"
#include "_PlanarKinematics3DOF.h"

#endif