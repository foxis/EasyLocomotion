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

#if !defined(PLANAR_KINEMATICS_2DOF_H)
#define PLANAR_KINEMATICS_2DOF_H

namespace Locomotion {

template<typename T> class _PlanarKinematics<T, 2> : public _KinematicsModel<T> {
public:
    const PlanarJoint_t<T> * config;
    const _ConstraintVolume<T> working_space

public:
    _PlanarKinematics(const PlanarJoint_t<T> * joints, const _ConstraintVolume<T> & working_space)
        : config(joints), working_space(working_space) {
    }
    ~_PlanarKinematics() {
    }

    /// Performs planar forward kinematics 
    /// assuming that the first joint is rotation about y axis
    ///
    virtual bool direct(const T * angle_arr, _Vector3D<T> &dst) {
        T a = angle_arr[1];
        _Vector3D<T> effector(config[0].length + config[1].length * cos(a), config[1].length * sin(a), 0);
        dst.x = effector.x * cos(angle_arr[0]);
        dst.y = effector.y;
        dst.z = effector.x * sin(angle_arr[0]));
        return true;
    }

    virtual T inverse(const _Vector3D<T> & target, const T * current_angle_arr, T * angle_arr, T eps, size_t max_iterations) {
        angle_arr[1] = config[1].constraints.limit(atan2(target.y, target.x - config[0].length));
        angle_arr[0] = config[0].constraints.limit(atan2(target.z, target.x));
        _Vector3D<T> pos;
        direct(angle_arr, pos);
        return (target - pos).magnitudeSqr();
    }
};

typedef _PlanarKinematics<real_t, 2> PlanarKinematics2DOF;

};

#endif