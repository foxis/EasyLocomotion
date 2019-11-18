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

#if !defined(KINEMATICS_IK_SOLVER_JACOB_H)
#define KINEMATICS_IK_SOLVER_JACOB_H

#include "locomotion.h"
#include "../math_utils.h"

namespace Locomotion {

template<typename T, size_t DOF>
bool ik_solver_jacobian_pos(const _Matrix<T, 4, 4> * H0i, const uint8_t * joint_types, const _Vector<T, 3> & v, T * param_arr) {
    _MatrixStatic<T, 3, DOF> J;
    _MatrixStatic<T, DOF, 3> Jinv;
    _VectorFromArr<T, DOF> params(param_arr);
    _Vector3D<T> d, di, dn;
    _Vector3D<T> R(0.0, 0.0, 1.0);

    H0i[DOF].get_col(dn, 3);

    for (size_t i = 0; i < DOF; i++) {
        H0i[i].get_col(R, 2);

        if (joint_types[i] == 0) {
            H0i[i].get_col(di, 3);
            dn.sub(d, di);
            R.cross(d, di);
            J.set_col(di, 0, i);
            J.set_col(R, 3, i);
        } else if (joint_types[i] == 3) {
            J.set_col(R, i);
        } else
            return false;
    }
    if (!J.pinv(Jinv))
        return false;
    J.mul(v, params);
    return true;
}

template<typename T, size_t DOF>
bool ik_solver_jacobian(const _Matrix<T, 4, 4> * H0i, const uint8_t * joint_types, const _Vector<T, 3> & v, const _Vector<T, 3> & w, T * param_arr) {
    _MatrixStatic<T, 6, DOF> J;
    _MatrixStatic<T, DOF, 6> Jinv;
    _VectorFromArr<T, DOF> params(param_arr);
    _VectorStatic<T, 6> vw;
    _Vector3D<T> d, di, dn;
    _Vector3D<T> R(0.0, 0.0, 1.0);

    vw.set_col(v);
    vw.set_col(w, 3);

    H0i[DOF].get_col(dn, 3);

    for (size_t i = 0; i < DOF; i++) {
        H0i[i].get_col(R, 2);

        if (joint_types[i] == 0) {
            H0i[i].get_col(di, 3);
            dn.sub(d, di);
            R.cross(d, di);
            J.set_col(di, 0, i);
            J.set_col(R, 3, i);
        } else if (joint_types[i] == 3) {
            J.set_col(R, i);
        } else
            return false;
    }
    if (!J.pinv(Jinv))
        return false;
    J.mul(vw, params);
    return true;
}

}

#endif
