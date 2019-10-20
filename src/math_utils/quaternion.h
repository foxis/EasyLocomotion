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

#if !defined(MATH_UTILS_QUATERNION_H)
#define MATH_UTILS_QUATERNION_H

#include "vectors.h"

namespace Locomotion {

template <class T> class _Quaternion : public _Vector4D<T> {
public:
	_Quaternion() : _Vector4D<T>() {}
	_Quaternion(T c) : _Vector4D<T>(c) {
	}
	_Quaternion(T x, T y, T z, T w) : _Vector4D<T>(x, y, z, w) {
	}
	_Quaternion(const T * arr) : _Vector4D<T>(arr) {
	}
	_Quaternion(const _Vector<T, 4> & v) : _Vector4D<T>(v) {
	}
	_Quaternion(const _Vector4D<T> & v) : _Vector4D<T>(v) {
	}
	_Quaternion(const _Vector<T, 3> & v) : _Vector4D<T>() {
		this->container._copy(v.container, 3);
		this->w = 1;
	}

	_Quaternion<T> product(const _Vector<T, 4>& q) const {
		return _Quaternion<T>(
			this->w*q.x + this->x*q.w + this->y*q.z - this->z*q.y,
			this->w*q.y - this->x*q.z + this->y*q.w + this->z*q.x,
			this->w*q.z + this->x*q.y - this->y*q.x + this->z*q.w,
			this->w*q.w - this->x*q.x - this->y*q.y - this->z*q.z
		);
	}

	_Quaternion<T> conjugate() const {
		return _Quaternion<T>(-this->x, -this->y, -this->z, this->w);
	}

	T getRoll() {
		return atan2f(this->w*this->x + this->y*this->z, 0.5f - this->x*this->x - this->y*this->y);
	}
	T getPitch() {
		return asinf(-2.0f * (this->x*this->z - this->w*this->y));
	}
	T getYaw() {
		return atan2f(this->x*this->y + this->w*this->z, 0.5f - this->y*this->y - this->z*this->z);
	}
	virtual void mul(const _Quaternion<T>& a, const _Quaternion<T>& b, _Quaternion<T> & q) const
	{
        q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
        q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
        q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
        q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
	}

	_Quaternion<T> operator * (const _Vector<T, 4> &b) const {
		_Quaternion<T> tmp;
		this->mul(*this, b, tmp);
		return tmp;
	}

	void operator *= (const _Vector<T, 4> &b) const {
		_Quaternion<T> tmp;
		this->mul(*this, b, tmp);
		*this = tmp;
	}

	_Quaternion<T> operator + (const _Vector<T, 4> & v) const {
		return _Quaternion<T>(this->x + v.data()[0], this->y + v.data()[1], this->z + v.data()[2], this->w + v.data()[3]);
	}
	_Quaternion<T> operator - (const _Vector<T, 4> & v) const {
		return _Quaternion<T>(this->x - v.data()[0], this->y - v.data()[1], this->z - v.data()[2], this->w - v.data()[3]);
	}
	_Quaternion<T> operator * (T c) const {
		return _Quaternion<T>(this->x * c, this->y * c, this->z * c, this->w * c);
	}
	_Quaternion<T> operator / (T c) const {
		return _Quaternion<T>(this->x / c, this->y / c, this->z / c, this->w / c);
	}

	void operator *= (T c) {
		this->x *= c;
		this->y *= c;
		this->z *= c;
		this->w *= c;
	}
	void operator /= (T c) {
		this->x /= c;
		this->y /= c;
		this->z /= c;
		this->w /= c;
	}

#define	SLERP_EPSILON 1.0E-10
	_Quaternion<T> slerp(const _Quaternion<T>& b, T time, T spin)
	{
		_Quaternion<T> tmp;
		T k1,k2;					// interpolation coefficions.
		T angle;					// angle between A and B
		T angleSpin;			// angle between A and B plus spin.
		T sin_a, cos_a;	// sine, cosine of angle
		int flipk2;						// use negation of k2.

		cos_a = dotunit( b );
		if (cos_a < 0.0)
		{
			cos_a = -cos_a;
			flipk2 = -1;
		}
		else
			flipk2 = 1;

		if ( (1.0 - cos_a) < SLERP_EPSILON )
		{
			k1 = 1.0 - time;
			k2 = time;
		}
		else
		{				/* normal case */
			angle = acos(cos_a);
			sin_a = sin(angle);
			angleSpin = angle + spin * M_PI;
			k1 = sin( angle - time*angleSpin ) / sin_a;
			k2 = sin( time*angleSpin ) / sin_a;
		}
		k2 *= flipk2;

		tmp.x = k1*this->x + k2*b.x;
		tmp.y = k1*this->y + k2*b.y;
		tmp.z = k1*this->z + k2*b.z;
		tmp.w = k1*this->w + k2*b.w;
		return tmp;
	}

	_Quaternion<T> slerplong( const _Quaternion<T>& b, T time, T spin )
	{
		_Quaternion<T> tmp;
		T k1,k2;					// interpolation coefficions.
		T angle;					// angle between A and B
		T angleSpin;			// angle between A and B plus spin.
		T sin_a, cos_a;	// sine, cosine of angle

		cos_a = dotunit( b );

		if (1.0 - fabs(cos_a) < SLERP_EPSILON) {
			k1 = 1.0 - time;
			k2 = time;
		} else {				/* normal case */
			angle = acos(cos_a);
			sin_a = sin(angle);
			angleSpin = angle + spin * M_PI;
			k1 = sin( angle - time * angleSpin ) / sin_a;
			k2 = sin( time*angleSpin ) / sin_a;
		}

		tmp.x = k1*this->x + k2*b.x;
		tmp.y = k1*this->y + k2*b.y;
		tmp.z = k1*this->z + k2*b.z;
		tmp.w = k1*this->w + k2*b.w;
		return tmp;
	}
#undef SLERP_EPSILON
};

typedef _Quaternion<real_t> Quaternion;
#define Quaternion_ZERO Quaternion(0,0,0,0)

};

#endif
