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

#if !defined(MATH_UTILS_BOUNDS_H)
#define MATH_UTILS_BOUNDS_H

#include <math.h>
#include "types.h"

namespace Locomotion {

template <class T> class _BoundingSegment {
public:
	_BoundingSegment() : _BoundingSegment<T>(_BoundingSegment<T>::ZERO) {

	}
	_BoundingSegment(const _BoundingSegment<T>& v)
	{
		copy(v);
	}
	_BoundingSegment(T min, T max)
	{
		this->min = min;
		this->max = max;
	}

	void copy(const _BoundingSegment<T>& v)
	{
		min = v.min;
		max = v.max;
	}

	void length() const {
		return fabs(this.max - this.min);
	}

	bool in(real_t val) const {
		return val >= min && val <= max;
	}

	T min, max;
};


template <class T> class _BoundingArea {
public:
	_BoundingArea() : _BoundingArea<T>(_BoundingArea<T>::ZERO) {

	}
	_BoundingArea(const _BoundingArea<T>& v)
	{
		copy(v);
	}
	_BoundingArea(const _Vector2D<T> & min, const _Vector2D<T> & max)
	{
		this->min = min;
		this->max = max;
	}

	void copy(const _BoundingArea<T>& v)
	{
		min = v.min;
		max = v.max;
	}

	void area() const {
		real_t a = max.x - min.x;
		real_t b = max.y - min.y;
		return a * b;
	}

	bool in(const _Vector2D<T> & val) const {
		return val.x >= min.x && val.x <= max.x && val.y >= min.y && val.y <= max.y;
	}

	_Vector2D<T> min, max;
};


template <class T> class _BoundingVolume {
public:
	_BoundingVolume() : _BoundingVolume<T>(_BoundingVolume<T>::ZERO) {

	}
	_BoundingVolume(const _BoundingVolume<T>& v)
	{
		copy(v);
	}
	_BoundingVolume(const _Vector3D<T> & min, const _Vector3D<T> & max)
	{
		this->min = min;
		this->max = max;
	}

	void copy(const _BoundingVolume<T>& v)
	{
		min = v.min;
		max = v.max;
	}

	void volume() const {
		real_t a = max.x - min.x;
		real_t b = max.y - min.y;
		real_t c = max.z - min.z;
		return a * b * c;
	}

	bool in(const _Vector3D<T> & val) const {
		return val.x >= min.x && val.x <= max.x 
				&& val.y >= min.y && val.y <= max.y
				&& val.z >= min.z && val.z <= max.z;
	}

	_Vector3D<T> min, max;
};

typedef _BoundingSegment<real_t> BoundingSegment;
typedef _BoundingArea<real_t> BoundingArea;
typedef _BoundingVolume<real_t> BoundingVolume;

};

#endif
