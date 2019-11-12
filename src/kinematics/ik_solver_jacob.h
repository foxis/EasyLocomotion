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
bool ik_solver_jacobian_pos(const _Matrix<T, 4, 4> * transforms, const _Vector<T, 3> & v, T * param_arr) {
    _MatrixStatic<T, 3, DOF> J;
    _MatrixStatic<T, DOF, 3> Jinv;
    _Matrix4x4<T> H0n = transforms[0];
    _VectorFromArr<T, DOF> params(param_arr);
    J.row(0)[0] = 0;
    J.row(1)[0] = 0;
    J.row(2)[0] = 1;
    for (size_t i = 1; i < DOF; i++) {
        J.row(0)[i] = H0n.row(0)[3];
        J.row(1)[i] = H0n.row(1)[3];
        J.row(2)[i] = H0n.row(2)[3];
        H0n = H0n * transforms[i];
    }
    if (!J.pinv(Jinv))
        return false;
    J.mul(v, params);
    return true;
}

template<typename T, size_t DOF>
bool ik_solver_jacobian(const _Matrix<T, 4, 4> * H, const T * joint_types, const _Vector<T, 3> & v, const _Vector<T, 3> & w, T * param_arr) {
    _MatrixStatic<T, 6, DOF> J;
    _MatrixStatic<T, DOF, 6> Jinv;
    _Matrix4x4<T> H0i[DOF + 1];
    _VectorFromArr<T, DOF> params(param_arr);
    _VectorStatic<T, 6> vw;
    _Vector3D<T> d, dn;
    _Vector3D<T> R(0, 0, 1);

    vw.set_col(v);
    vw.set_col(w, 3);

    H0i[0].eye();
    for (size_t i = 0; i < DOF; i++)
        H0i[i].mul_mat(H[i], H0i[i + 1]);
    H0i[DOF].get_col(dn, 3);

    for (size_t i = 0; i < DOF; i++) {
        H0i[i].get_col(R, 2);

        if (int(joint_types[i]) == 0) {
            // R x (dn - di)
            J.set_col(R, 3, i);
        } else if (int(joint_types[i]) == 3) {
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
