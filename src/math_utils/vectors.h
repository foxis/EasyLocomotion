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

#if !defined(MATH_UTILS_VECTORS_H)
#define MATH_UTILS_VECTORS_H

#include <math.h>
#include <functional>
#include "types.h"

namespace Locomotion {

///
/// Template class for general vectors 
///
template <typename T, size_t N> class _Vector {
protected:
	_DataContainerBase<T> &container;
public:
	_Vector(_DataContainerBase<T> &container) : container(container) {}
	_Vector(_DataContainerBase<T> &container, T c) : _Vector(container) {
		this->container.fill(c, N);
	}
	_Vector(_DataContainerBase<T> &container, const T * data) : _Vector(container) {
		this->container.copy(data, N);
	}
	_Vector(_DataContainerBase<T> &container, const _Vector<T, N> & v) : _Vector(container) {
		this->container.copy(v.container, N);
	}

	void operator = (const _Vector<T, N> & v) {
		container.copy(v.container, N);
	}
	/// will call a function on each element. basically doing Ai = f(Ai)
	void map(std::function<T (size_t idx, T val, void * params)> setter, void * params = NULL) {
		T *p = this->data();
		for (size_t i = 0; i < N; i++) {
			*p = setter(i, *p, params);
			++p;
		}
	}
	T magnitudeSqr() const {
		T sum = 0;
		const T *p = this->data();
		for (size_t i = 0; i < N; i++) {
			sum += *p * *p;
			++p;
		}
		return sum;
	}
	T magnitude() const {
		return sqrt(magnitudeSqr());
	}
	void normal(_Vector<T, N> & dst) const {
		T  magnitude = magnitude();
		const T *p = this->data();
		const T *p1 = dst->data();
		for (size_t i = 0; i < N; i++) {
			*p1 = *p / magnitude;
			++p;
			++p1;
		}

		dst.copy(*this);
		dst.normal();
	}
	void normal() {
		div(*this, magnitude());
	}

	void operator += (const _Vector<T, N> & v) {
		add(v);
	}
	void operator -= (const _Vector<T, N> & v) {
		sub(v);
	}
	void operator *= (T c) {
		mul(c);
	}
	void operator /= (T c) {
		div(c);
	}

	void add(const _Vector<T, N> & v) {
		add(*this, v);
	}
	static void add(_Vector<T, N> & dst, const _Vector<T, N> & v) {
		T *p = dst.data();
		const T *p1 = v.data();
		for (size_t i = 0; i < N; i++) {
			*p += *p1;
			++p1;
			++p;
		}
	}

	void sub(const _Vector<T, N> & v) {
		sub(*this, v);
	}
	static void sub(_Vector<T, N> & dst, const _Vector<T, N> & v) {
		T *p = dst.data();
		const T *p1 = v.data();
		for (size_t i = 0; i < N; i++) {
			*p += *p1;
			++p1;
			++p;
		}
	}

	void mul(T c) {
		mul(*this, c);
	}
	static void mul(_Vector<T, N> & dst, T c) {
		T *p = dst.data();
		for (size_t i = 0; i < N; i++) {
			*p *= c;
			++p;
		}
	}

	void div(T c) {
		div(*this, c);
	}
	static void div(_Vector<T, N> & dst, T c) {
		T *p = dst.data();
		for (size_t i = 0; i < N; i++) {
			*p /= c;
			++p;
		}
	}

	T dot(const _Vector<T, N> & v) const
	{
		T sum = 0;
		const T *p = this->data();
		const T *p1 = v.data();
		for (size_t i = 0; i < N; i++) {
			sum += *p * *p1;
			++p;
			++p1;
		}
		return sum;
	}

	static void linear_interpolate(const _Vector<T, N> & src, const _Vector<T, N> & dst, _Vector<T, N> & result, T t) {
		const T *p = src.data();
		const T *p1 = dst.data();
		T *pr = result.data();
		for (size_t i = 0; i < N; i++) {
			T d = *p1 - *p;
			*pr += d * t;
			++p;
			++p1;
			++pr;
		}
	}

	static void reflection(const _Vector<T, N> & n, const _Vector<T, N> & v, _Vector<T, N> & result) {
	    T N_dot_V = n.dot(v) * 2;
		const T *p = n.data();
		const T *p1 = v.data();
		T *pr = result.data();
		for (size_t i = 0; i < N; i++) {
			*pr = N_dot_V * *p - *p1;
			++p;
			++p1;
			++pr;
		}
	}	

	inline T* data() { return container._data(); }
	inline const T* data() const { return container._data(); }
};

template <typename T> class _Vector2D : public _Vector<T, 2>, private _DataContainerBase<T> {
	virtual T* _data() { return &x; }
	virtual const T* _data() const { return &x; }
    virtual _DataContainerBase<T> * clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerStatic<T, 2>();
        tmp->copy(&this->x, 2);
        return tmp;
    }
public:
	_Vector2D() : _Vector<T, 2>((_DataContainerBase<T>&)*this) {}
	_Vector2D(T c) : _Vector<T, 2>(*this, c) {}
	_Vector2D(T x, T y) : _Vector<T, 2>((_DataContainerBase<T>&)*this) {
		this->x = x;
		this->y = y;
	}
	_Vector2D(const T * data) : _Vector<T, 2>(*this, data) {}
	_Vector2D(const _Vector<T, 2> & v) : _Vector<T, 2>(*this, v) {}

	_Vector2D<T> operator + (const _Vector2D<T> & v) const {
		return _Vector2D<T>(x + v.x, y + v.y);
	}
	_Vector2D<T> operator - (const _Vector2D<T> & v) const {
		return _Vector2D<T>(x - v.x, y - v.y);
	}
	_Vector2D<T> operator * (real_t c) const {
		return _Vector2D<T>(x * c, y * c);
	}
	_Vector2D<T> operator / (real_t c) const {
		return _Vector2D<T>(x / c, y / c);
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

template <typename T> class _Vector3D : public _Vector<T, 3>, private _DataContainerBase<T> {
	virtual T* _data() { return &x; }
	virtual const T* _data() const { return &x; }
    virtual _DataContainerBase<T> * clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerStatic<T, 3>();
        tmp->copy(&this->x, 3);
        return tmp;
    }
public:
	_Vector3D() : _Vector<T, 3>((_DataContainerBase<T>&)*this) {}
	_Vector3D(T c) : _Vector<T, 3>(*this, c) {}
	_Vector3D(T x, T y, T z) : _Vector<T, 3>((_DataContainerBase<T>&)*this) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	_Vector3D(const T * data) : _Vector<T, 3>(*this, data) {}
	_Vector3D(const _Vector<T, 3> & v) : _Vector<T, 3>(*this, v) {}

	_Vector3D<T> operator + (const _Vector<T, 3> & v) const {
		return _Vector3D<T>(x + v.x, y + v.y, z + v.z);
	}
	_Vector3D<T> operator - (const _Vector<T, 3> & v) const {
		return _Vector3D<T>(x - v.x, y - v.y, z - v.z);
	}
	_Vector3D<T> operator * (T c) const {
		return _Vector3D<T>(x * c, y * c, z * c);
	}
	_Vector3D<T> operator / (T c) const {
		return _Vector3D<T>(x / c, y / c, z / c);
	}

	_Vector3D<T> cross(const _Vector<T, 3>& b) const
	{
 		return _Vector3D<T>(y * b.z - z * b.y,
			z * b.x - x * b.z,
			x * b.y - y * b.x);
	}

	_Vector3D<T> refraction(const _Vector<T, 3>& v, T ni, T nt)
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


template <typename T, size_t N> class _VectorStatic : public _Vector<T, N>, private _DataContainerStatic<T, N> {
public:
	_VectorStatic() : _Vector<T, N>((_DataContainerBase<T>&)*this) {}
	_VectorStatic(T c) : _Vector<T, N>(*this, c) {}
	_VectorStatic(const T * data) : _Vector<T, N>(*this, data) {}
	_VectorStatic(const _Vector<T, N> & v) : _Vector<T, N>(*this, v) {}
};

template <typename T, size_t N> class _VectorDynamic : public _Vector<T, N>, private _DataContainerDynamic<T, N> {
public:
	_VectorDynamic() : _Vector<T, N>((_DataContainerBase<T>&)*this) {}
	_VectorDynamic(T c) : _Vector<T, N>(*this, c) {}
	_VectorDynamic(const T * data) : _Vector<T, N>(*this, data) {}
	_VectorDynamic(const _Vector<T, N> & v) : _Vector<T, N>(*this, v) {}
};

typedef _Vector2D<real_t> Vector2D;
typedef _Vector3D<real_t> Vector3D;

#define EPSILON 0.001
#define Vector2D_ZERO Vector2D(0,0)
#define Vector3D_ZERO Vector3D(0,0,0)

#define Vector2D_ONE_X Vector2D(1,0)
#define Vector2D_ONE_Y Vector2D(0,1)
#define Vector3D_ONE_X Vector3D(1,0,0)
#define Vector3D_ONE_Y Vector3D(0,1,0)
#define Vector3D_ONE_Z Vector3D(0,0,1)

};

#endif
