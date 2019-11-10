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
#include "../math_utils.h"

namespace Locomotion {

#if !defined(IK_SOLVER_CCD) && !defined(IK_SOLVER_JACOBIAN)
#define IK_SOLVER_CCD
#endif

template<typename T> class _LimbKinematicsModel {
public:
    virtual _Vector3D<T> forward(const T * angle_arr) {
        _Vector3D<T> tmp;
        forward(angle_arr, tmp);
        return tmp;
    }
    virtual bool forward(const T * angle_arr, _Vector3D<T> &dst) = 0;
    virtual bool inverse(const _Vector3D<T> & target, const T * current_angle_arr, T * angle_arr, _Vector3D<T> & actual, T eps, size_t max_iterations) = 0;
    virtual size_t dof() const = 0;
    virtual _ConstraintVolume<T> working_space() const = 0;
};

typedef _LimbKinematicsModel<real_t> LimbKinematicsModel;

template<typename T>
struct _LimbConfig_t {
	_Vector3D<T> displacement;
	_Vector3D<T> orientation;
};

template<typename T, size_t LIMBS> class _BodyModel {
protected:
	_LimbKinematicsModel<T> **limbs;
	const _LimbConfig_t<T> *limb_config;
    _Matrix4x4<T> limb_transform_fw[LIMBS];
    _Matrix4x4<T> limb_transform_inv[LIMBS];

    typedef T* LimbArr_t;

public:
    // holders for joint angles for each limb
	LimbArr_t current_angles[LIMBS];
	LimbArr_t target_angles[LIMBS];

    // all coordinates are in body space
    _ConstraintVolume<T> working_space[LIMBS];

	_Vector3D<T> position;
	_Vector3D<T> orientation;

	_Vector3D<T> limb_pos[LIMBS];
	_Vector3D<T> actual_limb_pos[LIMBS];
	_Vector3D<T> reference_limb_pos[LIMBS];

public:
	_BodyModel(_LimbKinematicsModel<T> ** limbs, const _LimbConfig_t<T> * limb_config, const _Vector3D<T> & position, const _Vector3D<T> & orientation)
		: limbs(limbs), limb_config(limb_config), position(position), orientation(orientation) {
		for (size_t i = 0; i < LIMBS; i ++) {
            const size_t dof = limbs[i]->dof();
            current_angles[i] = new T[dof];
            target_angles[i] = new T[dof];
            memset(current_angles[i], 0, sizeof(T) * dof);
            memset(target_angles[i], 0, sizeof(T) * dof);
		}
        calculate_transforms();
	}
    ~_BodyModel() {
        for (size_t i = 0; i < LIMBS; i++) {
            delete[] current_angles[i];
            delete[] target_angles[i];
        }
    }

    ///
    /// Precalculate body->limb and limb->body transformation matrices
    /// and transform limb working space into body working space for each limb
    ///
    /// must be called each time limb configuration changes
    void calculate_transforms() {
        _Vector4D<T> vol_min_4, vol_max_4, tmp;
        _Vector3D<T> zero(0.0);
		for (size_t i = 0; i < LIMBS; i ++) {
            limb_transform_fw[i].homogenous(limb_config[i].orientation, limb_config[i].displacement, zero);
            limb_transform_fw[i].inverse(limb_transform_inv[i]);

            // TODO: Move into constraints
            // allow for other constraints than Volume
            tmp = limbs[i]->working_space().min;
            limb_transform_fw[i].mul(tmp, vol_min_4);
            tmp = limbs[i]->working_space().max;
            limb_transform_fw[i].mul(tmp, vol_max_4);
            working_space[i].min = vol_min_4.vector3d();
            working_space[i].max = vol_max_4.vector3d();
            working_space[i].sort();
		}
    }

    ///
    /// calculates a plane equation from three points
    /// aX + bY + zC + d = 0
    /// where a = result.x, b = result.y and so on
    void calculate_plane(const _Vector<T, 3> & a, const _Vector<T, 3> & b, const _Vector<T, 3> & c, _Vector<T, 4> & result) {
        _Vector3D<T> ac(c - a);
        _Vector3D<T> ab(b - a);
        _Vector3D<T> abac;
        ab.cross(ac, abac);
        result = abac;
        abac.w = -abac.dot(a);
    }

    ///
    /// given limbs that support the body, calculate limb positions
    /// relative to reference_limb_pos that would result in given
    /// position and orientation
    void inverse(const bool * support) {        
        _Matrix4x4<T> transform;
        transform.homogenous(orientation, position);

        for (size_t i = 0; i < LIMBS; i++)
            // TODO: not sure how this will work yet
            if (support[i])
                transform.mul(reference_limb_pos[i], limb_pos[i]);
    }

    ///
    /// given limbs that support the body, calculate body position 
    /// and orientation based on limb_pos relative to reference_limb_pos
    ///
    void forward(const bool * support) {

    }

    ///
    /// given current_angles for each limb calculate
    /// limb_pos in body reference frame
    ///
    virtual void forward_limbs() {
        _Vector4D<T> pos_4;
        _Vector4D<T> tmp;
        _Vector3D<T> pos_3;
        for (size_t i = 0; i < LIMBS; i++) {            
            limbs[i]->forward(current_angles[i], pos_3);
            tmp = pos_3;
            limb_transform_fw[i].mul(tmp, pos_4);
            limb_pos[i] = pos_4.vector3d();
        }
    }

    ///
    /// given limb_pos in body reference frame calculate 
    /// target_angles for each limb as well as actual_limb_pos
    virtual void inverse_limbs(const bool * support, T eps, size_t max_iterations) {
        _Vector3D<T> local_pos;
        _Vector4D<T> pos_4;
        _Vector4D<T> tmp;
        for (size_t i = 0; i < LIMBS; i++) {
            tmp = limb_pos[i];
            limb_transform_inv[i].mul(tmp, pos_4);
            local_pos = pos_4.vector3d();
            limbs[i]->inverse(local_pos, current_angles[i], target_angles[i], local_pos, eps, max_iterations);
            memcpy(current_angles[i], target_angles[i], sizeof(T) * limbs[i]->dof());
            tmp = local_pos;
            limb_transform_fw[i].mul(tmp, pos_4);
            actual_limb_pos[i] = pos_4.vector3d();
        }
    }
};

typedef _LimbConfig_t<real_t> LimbConfig_t;

template<size_t DOF>
using BodyModel = _BodyModel<real_t, DOF>;

}

#endif