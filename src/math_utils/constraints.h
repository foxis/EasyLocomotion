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

#if !defined(MATH_UTILS_CONSTRAINTS_H)
#define MATH_UTILS_CONSTRAINTS_H

#include <math.h>
#include "vectors.h"

namespace Locomotion {

template <class T> class _ConstraintSegment {
public:
	_ConstraintSegment() {
		min = max = 0;
	}
	_ConstraintSegment(const _ConstraintSegment<T>& v)
	{
		copy(v);
	}
	_ConstraintSegment(T min, T max)
	{
		this->min = min;
		this->max = max;
	}

	void copy(const _ConstraintSegment<T>& v)
	{
		min = v.min;
		max = v.max;
	}

	inline T length() const {
		return max - min;
	}

	inline bool in(T val) const {
		return val >= min && val <= max;
	}

	inline T limit(T val) const {
		if (val < min)
			return min;
		else if (val > max)
			return max;
		else 
			return val;
	}

	inline T middle() const {
		return (min + max) / 2.0;
	}

	void sort() {
		T mn, mx;
		mn = MIN(this->min, this->max);
		mx = MAX(this->min, this->max);
		this->min = mn;
		this->max = mx;
	}

	T min, max;
};


template <class T> class _ConstraintArea {
public:
	_ConstraintArea() {
		min.x = min.y = max.x = max.y = 0;
	}
	_ConstraintArea(const _ConstraintArea<T>& v)
	{
		copy(v);
	}
	_ConstraintArea(const _Vector2D<T> & min, const _Vector2D<T> & max)
	{
		this->min = min;
		this->max = max;
	}

	inline void copy(const _ConstraintArea<T>& v)
	{
		min = v.min;
		max = v.max;
	}

	inline T area() const {
		real_t a = max.x - min.x;
		real_t b = max.y - min.y;
		return a * b;
	}

	inline bool in(const _Vector2D<T> & val) const {
		return val.x >= min.x && val.x <= max.x && val.y >= min.y && val.y <= max.y;
	}

	inline _Vector2D<T> limit(const _Vector2D<T> & val) const {
		_Vector2D<T> tmp;
		if (val.x < min.x)
			tmp.x = min.x;
		else if (val.x > max.x)
			tmp.x = max.x;
		else
			tmp.x = val.x;
		if (val.y < min.y)
			tmp.y = min.y;
		else if (val.y > max.y)
			tmp.y = max.y;
		else
			tmp.y = val.y;
		return tmp;
	}

	inline _Vector2D<T> middle() const {
		return _Vector2D<T>((min.x + max.x) / 2.0, (min.y + max.y) / 2.0);
	}

	void sort() {
		T mn, mx;
		mn = MIN(this->min.x, this->max.x);
		mx = MAX(this->min.x, this->max.x);
		this->min.x = mn;
		this->max.x = mx;
		mn = MIN(this->min.y, this->max.y);
		mx = MAX(this->min.y, this->max.y);
		this->min.y = mn;
		this->max.y = mx;
	}

	_Vector2D<T> min, max;
};


template <class T> class _ConstraintVolume {
public:
	_ConstraintVolume() {
		min.x = min.y = min.z = max.x = max.y = max.z = 0;
	}
	_ConstraintVolume(const _ConstraintVolume<T>& v)
	{
		copy(v);
	}
	_ConstraintVolume(const _Vector3D<T> & min, const _Vector3D<T> & max)
	{
		this->min = min;
		this->max = max;
	}

	inline void copy(const _ConstraintVolume<T>& v)
	{
		min = v.min;
		max = v.max;
	}

	inline T volume() const {
		real_t a = max.x - min.x;
		real_t b = max.y - min.y;
		real_t c = max.z - min.z;
		return a * b * c;
	}

	inline bool in(const _Vector3D<T> & val) const {
		return val.x >= min.x && val.x <= max.x 
				&& val.y >= min.y && val.y <= max.y
				&& val.z >= min.z && val.z <= max.z;
	}

	inline _Vector3D<T> limit(const _Vector3D<T> & val) const {
		_Vector3D<T> tmp;
		if (val.x < min.x)
			tmp.x = min.x;
		else if (val.x > max.x)
			tmp.x = max.x;
		else
			tmp.x = val.x;
		if (val.y < min.y)
			tmp.y = min.y;
		else if (val.y > max.y)
			tmp.y = max.y;
		else
			tmp.y = val.y;
		if (val.z < min.z)
			tmp.z = min.z;
		else if (val.z > max.z)
			tmp.z = max.z;
		else
			tmp.z = val.z;
		return tmp;
	}

	inline _Vector3D<T> middle() const {
		return _Vector3D<T>((min.x + max.x) / 2.0, (min.y + max.y) / 2.0, (min.z + max.z) / 2.0);
	}

	void sort() {
		T mn, mx;
		mn = MIN(this->min.x, this->max.x);
		mx = MAX(this->min.x, this->max.x);
		this->min.x = mn;
		this->max.x = mx;
		mn = MIN(this->min.y, this->max.y);
		mx = MAX(this->min.y, this->max.y);
		this->min.y = mn;
		this->max.y = mx;
		mn = MIN(this->min.z, this->max.z);
		mx = MAX(this->min.z, this->max.z);
		this->min.z = mn;
		this->max.z = mx;
	}

	_Vector3D<T> min, max;
};

typedef _ConstraintSegment<real_t> ConstraintSegment;
typedef _ConstraintArea<real_t> ConstraintArea;
typedef _ConstraintVolume<real_t> ConstraintVolume;

};

#endif
