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
#define IK_SOLVER_JACOBIAN
#endif

///
/// Linked joint chain kinematics model base
/// T - type of joint parameter
template<typename T> class _LimbKinematicsModel {
public:
    virtual _Vector3D<T> forward(const T * param_arr) {
        _Vector3D<T> tmp;
        forward(param_arr, tmp);
        return tmp;
    }
    virtual bool forward(const T * param_arr, _Vector<T, 3> &dst) = 0;
    virtual bool inverse(const _Vector<T, 3> & target, const T * current_param_arr, T * param_arr, _Vector<T, 3> & actual, T eps, size_t max_iterations) = 0;
    virtual size_t dof() const = 0;
    virtual _ConstraintVolume<T> working_space() const = 0;
};

typedef _LimbKinematicsModel<real_t> LimbKinematicsModel;

///
/// Limb position and orientation relative to body center
///
template<typename T>
struct _LimbConfig_t {
	_Vector3D<T> displacement;
	_Vector3D<T> orientation;
};

///
/// Body kinematics model using limbs
/// T - type for mathematical operations
/// LIMBS - number of limbs
template<typename T, size_t LIMBS> class _BodyModel {
protected:
    typedef _LimbKinematicsModel<T> * LimbModelPtr_t;

	LimbModelPtr_t *limbs;
	const _LimbConfig_t<T> *limb_config;
    _Matrix4x4<T> limb_transform_fw[LIMBS];
    _Matrix4x4<T> limb_transform_inv[LIMBS];

public:
    // holders for joint parameters for each limb
	T *current_joints;
	T *target_joints;

    // all coordinates are in body space
    _ConstraintVolume<T> limb_working_space[LIMBS];

    /// body transform
    _ConstraintVolume<T> working_space;
    _Matrix4x4<T> transform;
    _Matrix4x4<T> transform_inv;

    /// limb state
	_Vector3D<T> limb_pos[LIMBS];
	_Vector3D<T> actual_limb_pos[LIMBS];
	_Vector3D<T> reference_limb_pos[LIMBS];

public:
	_BodyModel(LimbModelPtr_t * limbs, const _LimbConfig_t<T> * limb_config, const _ConstraintVolume<T> & working_space) 
        : _BodyModel(limbs, limb_config, working_space, _Vector3D<T>(0.0), _Vector3D<T>(0.0), NULL)
    {}
	_BodyModel(LimbModelPtr_t * limbs, const _LimbConfig_t<T> * limb_config, const _ConstraintVolume<T> & working_space, const _Vector3D<T> & position) 
        : _BodyModel(limbs, limb_config, working_space, position, _Vector3D<T>(0.0), NULL)
    {}
	_BodyModel(LimbModelPtr_t * limbs, const _LimbConfig_t<T> * limb_config, const _ConstraintVolume<T> & working_space, const _Vector3D<T> & position, const _Vector3D<T> & orientation) 
        : _BodyModel(limbs, limb_config, working_space, position, orientation, NULL)
    {}
	_BodyModel(LimbModelPtr_t * limbs, const _LimbConfig_t<T> * limb_config, const _ConstraintVolume<T> & working_space, const _Vector3D<T> & position, const _Vector3D<T> & orientation, const bool * supports)
		: limbs(limbs), limb_config(limb_config), working_space(working_space) {
        size_t total_joints = 0;
		for (size_t i = 0; i < LIMBS; i ++) {
            const size_t dof = limbs[i]->dof();
            total_joints += dof;
		}
        current_joints = new T[total_joints];
        target_joints = new T[total_joints];
        memset(current_joints, 0, sizeof(T) * total_joints);
        memset(target_joints, 0, sizeof(T) * total_joints);
        set_transform(position, orientation);
        calculate_transforms();
        forward_limbs();
        fix_reference();
        inverse(NULL, 1e-5, 10);
	}
    ~_BodyModel() {
        delete[] current_joints;
        delete[] target_joints;
    }

    void set_transform(const _Vector3D<T> & position, const _Vector3D<T> & orientation) {
        _Vector3D<T> zero(0.0);
        transform.homogenous(orientation, position, zero);
        transform.homogenous_inverse(transform_inv);
    }
    void set_position(const _Vector3D<T> & position) {
        transform.set_translation(working_space.limit(position));
        transform.homogenous_inverse(transform_inv);
    }
    void set_orientation(const _Vector3D<T> & orientation) {
        _Matrix3x3<T> R;
        R.rotation(orientation);
        set_orientation(R);
    }
    void set_orientation(const _Matrix<T, 3, 3> & R) {
        transform.set_rotation(R);
        transform.homogenous_inverse(transform_inv);
    }

    void fix_reference(const bool * support = NULL) {
        for (size_t i = 0; i < LIMBS; i++)
            if (support == NULL || !support[i])
                reference_limb_pos[i] = limb_pos[i];
    }
    void fix_reference_actual(const bool * support = NULL) {
        for (size_t i = 0; i < LIMBS; i++)
            if (support == NULL || !support[i])
                reference_limb_pos[i] = actual_limb_pos[i];
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
            limb_transform_fw[i].homogenous_inverse(limb_transform_inv[i]);

            // TODO: Move into constraints
            // allow for other constraints than Volume
            tmp = limbs[i]->working_space().min;
            limb_transform_fw[i].mul(tmp, vol_min_4);
            tmp = limbs[i]->working_space().max;
            limb_transform_fw[i].mul(tmp, vol_max_4);
            vol_min_4.vector3d(limb_working_space[i].min);
            vol_max_4.vector3d(limb_working_space[i].max);
            limb_working_space[i].sort();
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
    void inverse(const bool * support, T eps, size_t max_iterations) {
        _Vector4D<T> ref, pos;
        for (size_t i = 0; i < LIMBS; i++)
            // TODO: not sure how this will work yet
            if (support == NULL || support[i]) {
                ref = reference_limb_pos[i];
                transform_inv.mul(ref, pos);
                pos.vector3d(limb_pos[i]);
            }
        inverse_limbs(eps, max_iterations);
    }

    ///
    /// given limbs that support the body, calculate body position 
    /// and orientation based on limb_pos relative to reference_limb_pos
    ///
    void forward(const bool * support) {

    }

    ///
    /// given current_joints for each limb calculate
    /// limb_pos in body reference frame
    ///
    virtual void forward_limbs() {
        _Vector4D<T> pos_4;
        _Vector4D<T> tmp;
        _Vector3D<T> pos_3;
        size_t joint_offs = 0;
        for (size_t i = 0; i < LIMBS; i++) {            
            limbs[i]->forward(current_joints + joint_offs, pos_3);
            tmp = pos_3;
            limb_transform_fw[i].mul(tmp, pos_4);
            limb_pos[i] = pos_4.vector3d();
            joint_offs += limbs[i]->dof();
        }
    }

    ///
    /// given limb_pos in body reference frame calculate 
    /// target_joints for each limb as well as actual_limb_pos
    virtual void inverse_limbs(T eps, size_t max_iterations) {
        _Vector3D<T> local_pos;
        _Vector4D<T> pos_4;
        _Vector4D<T> tmp;
        size_t joint_offs = 0;
        for (size_t i = 0; i < LIMBS; i++) {
            tmp = limb_pos[i];
            limb_transform_inv[i].mul(tmp, pos_4);
            local_pos = pos_4.vector3d();
            limbs[i]->inverse(local_pos, current_joints + joint_offs, target_joints + joint_offs, local_pos, eps, max_iterations);
            memcpy(current_joints + joint_offs, target_joints + joint_offs, sizeof(T) * limbs[i]->dof());
            tmp = local_pos;
            limb_transform_fw[i].mul(tmp, pos_4);
            actual_limb_pos[i] = pos_4.vector3d();
            joint_offs += limbs[i]->dof();
        }
    }
};

typedef _LimbConfig_t<real_t> LimbConfig_t;

template<size_t DOF>
using BodyModel = _BodyModel<real_t, DOF>;

}

#endif