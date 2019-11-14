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

#if !defined(DENAVIT_HARTENBERG_KINEMATICS_H)
#define DENAVIT_HARTENBERG_KINEMATICS_H

#include "../locomotion.h"
#include "kinematics.h"
#include "ik_solver_jacob.h"

namespace Locomotion {

///
/// FK and IK using Denavit-Hartenberg parameter table for homogenious transform matrix calculation.
/// How to construct DH parameter table:
/// https://www.youtube.com/watch?v=D3w3ZANOy3s (one prizmatic joint)
/// https://www.youtube.com/watch?v=dASdcqgBlqw (only rotational joints)
/// https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
///
/// DH table format:
///
///  n | theta | alpha | r | d | parameter index |
///  1 |
///
/// n - frame index
/// theta - rotation around Z previous axis
/// alpha - rotation around X previous axis
/// r - displacement along X axis
/// d - displacement along Z axis
/// parameter index - index of the column that joint parameter relates to (either 0 or 3 for rotational and prizmatic respectively)
template<typename T, size_t DOF> class _DenavitHartenbergKinematics : public _LimbKinematicsModel<T> {
public:
    const T * parameters;
    T joint_types[DOF];
    const _ConstraintSegment<T> * constraints;
    const _ConstraintVolume<T>& _working_space;
    _Matrix4x4<T> H0i[DOF + 1];

    bool calculate_transform(size_t frame, T parameter, _Matrix<T, 4, 4> & H) {
        T * p = H.data();
        const T * pparam = this->parameters + frame * 5;
        T theta = pparam[0];
        T alpha = pparam[1];
        T r = pparam[2];
        T d = pparam[3];
        parameter = this->constraints[frame].limit(parameter);
        switch (int(pparam[4])) {
            case 0: theta += parameter; break;
            case 1: alpha += parameter; break;
            case 2: r += parameter; break;
            case 3: d += parameter; break;
            default: return false;
        }
        T ct = cos(theta);
        T st = sin(theta);
        T ca = cos(alpha);
        T sa = sin(alpha);

        *(p++) = ct;
        *(p++) = -st * ca;
        *(p++) = st * sa;
        *(p++) = r * ct;
        *(p++) = st;
        *(p++) = ct * ca;
        *(p++) = -ct * sa;
        *(p++) = r * st;
        *(p++) = 0;
        *(p++) = sa;
        *(p++) = ca;
        *(p++) = d;
        *(p++) = 0;
        *(p++) = 0;
        *(p++) = 0;
        *(p++) = 1;
        return true;
    }
public:
    _DenavitHartenbergKinematics(const T * DH_parameters, const _ConstraintSegment<T> * param_constraints, const _ConstraintVolume<T> & working_space)
        : parameters(DH_parameters), constraints(param_constraints), _working_space(working_space) {
        for (size_t i = 0; i < DOF; i++)
            this->joint_types[i] = DH_parameters[I * 5 + 4];
    }
    ~_DenavitHartenbergKinematics() {
    }

    /// Performs planar forward kinematics 
    /// assuming that the first joint is rotation about y axis
    ///
    virtual bool forward(const T * param_arr, _Vector<T, 3> & effector) {
        _Matrix4x4<T> H;
        H0i[0].eye();
        for (size_t i = 0; i < DOF; i++) {
            if (!calculate_transform(i, param_arr[i], H))
                return false;
            H0i[i].mul_mat(H, H0i[i + 1]);
        }

        return forward(H0i[DOF], param_arr, effector);
    }
    bool forward(const _Matrix<T, 4, 4> & H0n, const T * param_arr, _Vector<T, 3> & effector) {
        H0n.get_col(effector, 3);
        return true;
    }

    virtual bool inverse(const _Vector<T, 3> & target, const T * current_param_arr, T * param_arr, _Vector<T, 3> & actual, T eps, size_t max_iterations) {
        T eps2 = SQR(eps);
        _Vector3D<T> error;
        for (size_t iter = 0; iter < max_iterations + 1; iter++) {
            if (!forward(current_param_arr, actual))
                return false;
            target.sub(actual, error);
            if (error.magnitudeSqr() < eps2)
                return true;
            if (iter == max_iterations)
                return false;
            if (!ik_solver_jacobian_pos<T, DOF>(this->H0i, this->joint_types, error, param_arr))
                return false;
        }

        return false;
    }

    virtual size_t dof() const { return DOF; }
    virtual _ConstraintVolume<T> working_space() const { return _working_space; }
};

};


#endif