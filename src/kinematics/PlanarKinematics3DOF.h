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

template<typename T> class _PlanarKinematics<T, 3> : public _KinematicsModel<T> {
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
        real_t a = angle_arr[1];
        real_t b = angle_arr[2] + a;
        _Vector3D<T> effector(
            config[0].length + config[1].length * cos(a) + config[2].length * cos(b), 
            config[1].length * sin(a) + config[2].length * sin(b), 
            0);
        dst.x = effector.x * cos(angle_arr[0]);
        dst.y = effector.y;
        dst.z = effector.x * sin(angle_arr[0]);
        return true;
    }

    virtual T inverse(const _Vector3D<T> & target, const T * current_angle_arr, T * angle_arr, T eps, size_t max_iterations) {

    }
};

typedef _PlanarKinematics<real_t, 3> PlanarKinematics3DOF;

};

#endif