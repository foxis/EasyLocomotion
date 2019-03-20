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

	_Quaternion(const _Quaternion<T> & q)
	{
		x = q.x;
		y = q.y;
		z = q.z;
		w = q.w;
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
	T getRoll() {
		return atan2f(w*x + y*z, 0.5f - x*x - y*y);
	}
	T getPitch() {
		return asinf(-2.0f * (x*z - w*y));
	}
	T getYaw() {
		return atan2f(x*y + w*z, 0.5f - y*y - z*z);
	}
	T dot(const _Quaternion<T>& pt) const
	{
		return (pt.x*x + pt.y*y + pt.z*z + pt.w*w)/(getMagnitude()*pt.getMagnitude());
	}
	T dotunit(const _Quaternion<T>& pt) const
	{
		return pt.x*x + pt.y*y + pt.z*z + pt.w*w;
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

	T dot(const _Vector2D<T>& pt) const
	{
		return pt.x * x + pt.y * y;
	}

	_Vector2D<T> linear_interpolate(const _Vector2D<T>& v, T t) const
	{
		return _Vector2D<T>(
			x + (v.x - x) * t,
			y + (v.y - y) * t);
	}

	_Vector2D<T> reflection(const _Vector2D<T>& v)
	{
	    T   N_dot_V;
	    _Vector2D<T> rfl;

	    N_dot_V = dot(v) * 2;
	    rfl.x = (N_dot_V * x) - v.x;
	    rfl.y = (N_dot_V * y) - v.y;

	    return rfl;
	}

	_Vector2D<T> refraction(const _Vector2D<T>& v, T ni, T nt)
	{
    _Vector2D<T> _T;      /* the refracted vector */
    _Vector2D<T> sin_T;      /* sin vect of the refracted vect */
    _Vector2D<T> cos_V;      /* cos vect of the incident vect */
    T len_sin_T;  /* length of sin T squared */
    T n_mult;     /* ni over nt */
    T N_dot_V;
    T N_dot_T;

	  if ( (N_dot_V = dot(v)) > 0.0 )
			n_mult = ni / nt;
		else
			n_mult = nt / ni;
		cos_V.x = x * N_dot_V;
		cos_V.y = y * N_dot_V;
		sin_T.x = (cos_V.x - v.x) * (n_mult);
		sin_T.y = (cos_V.y - v.y) * (n_mult);
		if ( (len_sin_T = sin_T.dot(sin_T)) >= 1.0 )
	        return _T;    /* internal reflection */
	    N_dot_T = sqrt(1.0 - len_sin_T);
	    if ( N_dot_V < 0.0 )
			N_dot_T = -N_dot_T;
	    _T.x = sin_T.x - x * N_dot_T;
	    _T.y = sin_T.y - y * N_dot_T;
	    return _T;
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

	T dot(const _Vector3D<T>& pt) const
	{
		return pt.x * x + pt.y * y + pt.z * z;
	}
	_Vector3D<T> cross(const _Vector3D<T>& b) const
	{
 		return _Vector3D<T>(y * b.z - z * b.y,
			z * b.x - x * b.z,
			x * b.y - y * b.x);
	}
	_Vector3D<T> linear_interpolate(const _Vector3D<T>& v, T t) const
	{
		return _Vector3D<T>(
			x + (v.x - x) * t,
			y + (v.y - y) * t,
			z + (v.z - z) * t);
	}

	_Vector3D<T> reflection(const _Vector3D<T>& v)
	{
	    T   N_dot_V;
	    _Vector3D<T> rfl;

	    N_dot_V = dot(v) * 2;
	    rfl.x = (N_dot_V * x) - v.x;
	    rfl.y = (N_dot_V * y) - v.y;
	    rfl.z = (N_dot_V * z) - v.z;

	    return rfl;
	}

	_Vector3D<T> refraction(const _Vector3D<T>& v, T ni, T nt)
	{
    _Vector3D<T> _T;      /* the refracted vector */
    _Vector3D<T> sin_T;      /* sin vect of the refracted vect */
    _Vector3D<T> cos_V;      /* cos vect of the incident vect */
    T len_sin_T;  /* length of sin T squared */
    T n_mult;     /* ni over nt */
    T N_dot_V;
    T N_dot_T;

    if ( (N_dot_V = dot(v)) > 0.0 )
			n_mult = ni / nt;
		else
			n_mult = nt / ni;
		cos_V.x = x * N_dot_V;
		cos_V.y = y * N_dot_V;
		cos_V.z = z * N_dot_V;
		sin_T.x = (cos_V.x - v.x) * (n_mult);
		sin_T.y = (cos_V.y - v.y) * (n_mult);
		sin_T.z = (cos_V.z - v.z) * (n_mult);
		if ( (len_sin_T = sin_T.dot(sin_T)) >= 1.0 )
      return _T;    /* internal reflection */
    N_dot_T = sqrt(1.0 - len_sin_T);
    if ( N_dot_V < 0.0 )
		N_dot_T = -N_dot_T;
    _T.x = sin_T.x - x * N_dot_T;
    _T.y = sin_T.y - y * N_dot_T;
    _T.z = sin_T.z - z * N_dot_T;
    return _T;
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
