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

#if !defined(PLANAR_KINEMATICS_3DOF_H)
#define PLANAR_KINEMATICS_3DOF_H

namespace Locomotion {

template<typename T> class _PlanarKinematics<T, 3> : public _LimbKinematicsModel<T> {
public:
    const _PlanarJoint_t<T> * config;
    const _ConstraintVolume<T> & working_space;

public:
    _PlanarKinematics(const _PlanarJoint_t<T> * joints, const _ConstraintVolume<T> & working_space)
        : config(joints), working_space(working_space) {
    }
    ~_PlanarKinematics() {
    }

    /// Performs planar forward kinematics 
    /// assuming that the first joint is rotation about y axis
    ///
    virtual bool forward(const T * angle_arr, _Vector3D<T> & dst) {
        const T a0 = config[0].constraints.limit(angle_arr[0]);
        const T a1 = config[1].constraints.limit(angle_arr[1]);
        const T a2 = config[2].constraints.limit(angle_arr[2]) + a1;
        const T l0 = config[0].length;
        const T l1 = config[1].length;
        const T l2 = config[2].length;

        _Vector3D<T> effector(
            l0 + l1 * cos(a1) + l2 * cos(a2), 
            l1 * sin(a1) + l2 * sin(a2), 
            0);
        dst.x = effector.x * cos(a0);
        dst.y = effector.y;
        dst.z = effector.x * sin(a0);
        return true;
    }

    virtual bool inverse(const _Vector3D<T> & target, const T * current_angle_arr, T * angle_arr, _Vector3D<T> & actual, T eps, size_t max_iterations) {
        const T l0 = config[0].length;
        const T l1 = config[1].length;
        const T l2 = config[2].length;
        T x_prime = sqrt(SQR(target.x) + SQR(target.z)) - l0;
        const T gamma = atan2(target.y, x_prime);
        const T beta = acos((SQR(l1) + SQR(l2) - SQR(x_prime) - SQR(target.y)) / (2 * l1 * l2));
        const T alpha = acos((SQR(x_prime) + SQR(target.y) + SQR(l1) - SQR(l2)) / (2 * l1 * sqrt(SQR(target.y) + SQR(x_prime))));

        angle_arr[0] = config[0].constraints.limit(atan2(target.z, target.x));
        angle_arr[1] = gamma + alpha;
        angle_arr[2] = beta - M_PI;

        forward(angle_arr, actual);
        return true;
    }

    virtual size_t dof() const { return 3; }
    virtual _ConstraintVolume<T> target_space() const { return working_space; }
};

typedef _PlanarKinematics<real_t, 3> PlanarKinematics3DOF;

};

#endif