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

#if !defined(MATH_UTILS_H)
#define MATH_UTILS_H

#include <math.h>

namespace Locomotion {

template <class T> class _Quaternion {
    public:
        T w;
        T x;
        T y;
        T z;

        _Quaternion() : _Quaternion(0, 0, 0, 1) {
        }

        _Quaternion(T nx, T ny, T nz, T nw) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
        }

        _Quaternion<T> getProduct(const _Quaternion<T>& q) const {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            return _Quaternion<T>(
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w,
								w*q.w - x*q.x - y*q.y - z*q.z  // new w
							); // new z
        }

        _Quaternion<T> getConjugate() const {
            return _Quaternion<T>(-x, -y, -z, w);
        }

        T getMagnitude() const {
            return sqrt(w*w + x*x + y*y + z*z);
        }

        void normalize() {
            T m = getMagnitude();
            w /= m;
            x /= m;
            y /= m;
            z /= m;
        }

        _Quaternion<T> getNormalized() const {
            _Quaternion<T> r(x, y, z, w);
            r.normalize();
            return r;
        }
};


template <class T> class _Vector2D {
public:
	_Vector2D() : _Vector2D<T>(_Vector2D<T>::ZERO) {

	}
	_Vector2D(const _Vector2D<T>& v)
	{
		copy(v);
	}
	_Vector2D(T x, T y)
	{
		this->x = x;
		this->y = y;
	}

	T getMagnitude() const {
			return sqrt(x*x + y*y);
	}

	void normalize() {
			T m = getMagnitude();
			x /= m;
			y /= m;
	}

	void copy(const _Vector2D<T>& v)
	{
		x = v.x;
		y = v.y;
	}

	T x, y;
};


template <class T> class _Vector3D {
public:
	_Vector3D() : _Vector3D<T>(_Vector3D<T>::ZERO) {

	}
	_Vector3D(const _Vector3D<T>& v)
	{
		copy(v);
	}
	_Vector3D(T x, T y, T z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	T getMagnitude() const {
			return sqrt(x*x + y*y + z*z);
	}

	void normalize() {
			T m = getMagnitude();
			x /= m;
			y /= m;
			z /= m;
	}


	void copy(const _Vector3D<T>& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
	}

	T x, y, z;
};

#ifndef __AVR__
	typedef double real_t;
#else
	typedef float real_t;
#endif

typedef _Vector2D<real_t> Vector2D;
typedef _Vector3D<real_t> Vector3D;
typedef _Quaternion<real_t> Quaternion;
//typedef _Vector4D<common_type_t> Vector4D;

#define EPSILON 0.001
#define Vector2D_ZERO Vector2D(0,0)
#define Vector3D_ZERO Vector3D(0,0,0)
#define Quaternion_ZERO Quaternion(0,0,0,0)

#define Vector2D_ONE_X Vector2D(1,0)
#define Vector2D_ONE_Y Vector2D(0,1)
#define Vector3D_ONE_X Vector3D(1,0,0)
#define Vector3D_ONE_Y Vector3D(0,1,0)
#define Vector3D_ONE_Z Vector3D(0,0,1)

};

#endif
