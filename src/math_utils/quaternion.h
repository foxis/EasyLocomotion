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

template <class T> class _Quaternion : public _Vector<T, 4>, private _DataContainerBase<T> {
	virtual T* _data() { return &x; }
	virtual const T* _data() const { return &x; }
    virtual _DataContainerBase<T> * clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerStatic<T, 4>();
        tmp->copy(&this->x, 4);
        return tmp;
    }
public:
	_Quaternion() : _Quaternion(0, 0, 0, 1) {}
	_Quaternion(T c) : _Vector<T, 4>(*this, c) {}
	_Quaternion(T x, T y, T z, T w) : _Vector<T, 4>((_DataContainerBase<T>&)*this) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}
	_Quaternion(const T * data) : _Vector<T, 4>(*this, data) {}
	_Quaternion(const _Vector<T, 4> & v) : _Vector<T, 4>(*this, v) {}

	_Quaternion<T> getProduct(const _Vector<T, 4>& q) const {
		return _Quaternion<T>(
			w*q.x + x*q.w + y*q.z - z*q.y,
			w*q.y - x*q.z + y*q.w + z*q.x,
			w*q.z + x*q.y - y*q.x + z*q.w,
			w*q.w - x*q.x - y*q.y - z*q.z
		);
	}

	_Quaternion<T> getConjugate() const {
		return _Quaternion<T>(-x, -y, -z, w);
	}

	_Quaternion<T> getNormalized() const {
		_Quaternion<T> r(x, y, z, w);
		r.normalize();
		return r;
	}

	T getRoll() {
		return atan2f(w*x + y*z, 0.5f - x*x - y*y);
	}
	T getPitch() {
		return asinf(-2.0f * (x*z - w*y));
	}
	T getYaw() {
		return atan2f(x*y + w*z, 0.5f - y*y - z*z);
	}
	void mul(const _Quaternion<T>& b)
	{
		_Quaternion<T> q;
        q.x = w*b.x + x*b.w + y*b.z - z*b.y;
        q.y = w*b.y - x*b.z + y*b.w + z*b.x;
        q.z = w*b.z + x*b.y - y*b.x + z*b.w;
        q.w = w*b.w - x*b.x - y*b.y - z*b.z;
		*this = q;
	}
	_Quaternion<T> mul(const _Quaternion<T>& b) const
	{
		_Quaternion<T> tmp(*this);
		tmp.mul(b);
		return tmp;
	}
	_Quaternion<T> operator*(const _Quaternion<T>& b) const
	{
		_Quaternion<T> tmp(*this);
		tmp.mul(b);
		return tmp;
	}
	_Quaternion<T>& operator*=(const _Quaternion<T>& b)
	{
		mul(b);
		return *this;
	}

	_Quaternion<T> linear_interpolate(const _Quaternion<T>& v, T t) const
	{
		return _Quaternion<T>(
			x + (v.x - x) * t,
			y + (v.y - y) * t,
			z + (v.z - z) * t,
			w + (v.w - w) * t);
	}

#define	SLERP_EPSILON 1.0E-10
	_Quaternion<T> slerp(const _Quaternion& b, T time, T spin)
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

		tmp.x = k1*x + k2*b.x;
		tmp.y = k1*y + k2*b.y;
		tmp.z = k1*z + k2*b.z;
		tmp.w = k1*w + k2*b.w;
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

		tmp.x = k1*x + k2*b.x;
		tmp.y = k1*y + k2*b.y;
		tmp.z = k1*z + k2*b.z;
		tmp.w = k1*w + k2*b.w;
		return tmp;
	}
#undef SLERP_EPSILON

	T x, y, z, w;
};

typedef _Quaternion<real_t> Quaternion;
#define Quaternion_ZERO Quaternion(0,0,0,0)

};

#endif
