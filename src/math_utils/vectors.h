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
	_Vector(_DataContainerBase<T> &cont) : container(cont) 
	{
	}
	_Vector(_DataContainerBase<T> &cont, T c) : _Vector(cont) {
		this->container._fill(c, N);
	}
	_Vector(_DataContainerBase<T> &cont, const T * arr) : _Vector(cont) {
		this->container._copy(arr, N);
	}
	_Vector(_DataContainerBase<T> &cont, const _Vector<T, N> & v) : _Vector(cont) {
		this->container._copy(v.container, N);
	}

	void operator = (const _Vector<T, N> & v) {
		this->container._copy(v.container, N);
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
		dst = *this;
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
	virtual void add(_Vector<T, N> & dst, const _Vector<T, N> & v) const {
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
	virtual void sub(_Vector<T, N> & dst, const _Vector<T, N> & v) const {
		T *p = dst.data();
		const T *p1 = v.data();
		for (size_t i = 0; i < N; i++) {
			*p -= *p1;
			++p1;
			++p;
		}
	}

	void mul(T c) {
		mul(*this, c);
	}
	virtual void mul(_Vector<T, N> & dst, T c) const {
		T *p = dst.data();
		for (size_t i = 0; i < N; i++) {
			*p *= c;
			++p;
		}
	}

	void div(T c) {
		div(*this, c);
	}
	virtual void div(_Vector<T, N> & dst, T c) const {
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

	template<size_t KERNEL_SIZE, size_t OUTPUT_SIZE>
	void convolve(const _Vector<T, KERNEL_SIZE> & kernel, _Vector<T, OUTPUT_SIZE> & dst) const {
        static_assert(KERNEL_SIZE <= N, "Kernel size must be less or equal to the vector size");
        static_assert(OUTPUT_SIZE <= N, "Output size must be less or equal to the vector size");
        static_assert(OUTPUT_SIZE >= N - KERNEL_SIZE + 1, "Output size must be greather than vector size - kernel size");
        static_assert((OUTPUT_SIZE - (N - KERNEL_SIZE + 1)) % 2 == 0, "Padding must be equal on both sides");
		T * out = dst.data();
		const size_t PADDING = (OUTPUT_SIZE - (N - KERNEL_SIZE + 1)) / 2;

		// calculate head
		for (size_t i = KERNEL_SIZE - PADDING - 1; i < KERNEL_SIZE - 1; i++) {
			const T * in = this->data();
			const T * pk = kernel.data() + i;
			T sum = 0;
			for (size_t j = i; j < KERNEL_SIZE; j++) {
				sum += *in * *pk;
				++in;
				++pk;
			}
			*out = sum;
			++out;
		}
		// calculate body
		for (size_t i = 0; i < N - KERNEL_SIZE + 1; i++) {
			const T * in = this->data() + i;
			const T * pk = kernel.data();
			T sum = 0;
			for (size_t j = 0; j < KERNEL_SIZE; j++) {
				sum += *in * *pk;
				++in;
				++pk;
			}
			*out = sum;
			++out;
		}
		// calculate tail
		for (size_t i = N - KERNEL_SIZE + 1; i < N - KERNEL_SIZE + 1 + PADDING; i++) {
			const T * in = this->data() + i;
			const T * pk = kernel.data();
			T sum = 0;
			for (size_t j = i; j < N; j++) {
				sum += *in * *pk;
				++in;
				++pk;
			}
			*out = sum;
			++out;
		}
	}

	void linear_interpolate(const _Vector<T, N> & dst, _Vector<T, N> & result, T t) const {
		linear_interpolate(*this, dst, result, t);
	}
	static void linear_interpolate(const _Vector<T, N> & src, const _Vector<T, N> & dst, _Vector<T, N> & result, T t) {
		const T *p = src.data();
		const T *p1 = dst.data();
		T *pr = result.data();
		for (size_t i = 0; i < N; i++) {
			T d = *p1 - *p;
			*pr = *p + d * t;
			++p;
			++p1;
			++pr;
		}
	}

	void reflection(const _Vector<T, N> & v, _Vector<T, N> & result) const {
		reflection(*this, v, result);
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

template <typename T> class _Vector2D : public _Vector<T, 2>, protected _DataContainerBase<T> {
	virtual T* _data() { return &x; }
	virtual const T* _data() const { return &x; }
    virtual _DataContainerBase<T> * _clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerStatic<T, 2>();
        tmp->_copy(&this->x, 2);
        return tmp;
    }
public:
	_Vector2D() : _Vector<T, 2>(*(_DataContainerBase<T>*)this) {}
	_Vector2D(T c) : _Vector<T, 2>(*(_DataContainerBase<T>*)this) {
		this->_fill(c, 2);
	}
	_Vector2D(T x, T y) : _Vector<T, 2>(*(_DataContainerBase<T>*)this) {
		this->x = x;
		this->y = y;
	}
	_Vector2D(const T * arr) : _Vector<T, 2>(*(_DataContainerBase<T>*)this) {
		this->_copy(arr, 2);
	}
	_Vector2D(const _Vector<T, 2> & v) : _Vector<T, 2>(*(_DataContainerBase<T>*)this) {
		this->_copy(v.container, 2);
	}
	_Vector2D(const _Vector2D<T> & v) : _Vector<T, 2>(*(_DataContainerBase<T>*)this) {
		this->_copy(v.container, 2);
	}

	_Vector2D<T> operator + (const _Vector<T, 2> & v) const {
		return _Vector2D<T>(x + v.data()[0], y + v.data()[1]);
	}
	_Vector2D<T> operator - (const _Vector<T, 2> & v) const {
		return _Vector2D<T>(x - v.data()[0], y - v.data()[1]);
	}
	_Vector2D<T> operator * (real_t c) const {
		return _Vector2D<T>(x * c, y * c);
	}
	_Vector2D<T> operator / (real_t c) const {
		return _Vector2D<T>(x / c, y / c);
	}

	bool refraction(const _Vector<T, 2>& v, T ni, T nt, _Vector<T, 2> & result) const {
		return refraction(*this, v, ni, nt, result);
	}
	static bool refraction(const _Vector<T, 2>& normal, const _Vector<T, 2>& v, T ni, T nt, _Vector<T, 2> & result)
	{
		_Vector2D<T> sin_T;      /* sin vect of the refracted vect */
		_Vector2D<T> cos_V;      /* cos vect of the incident vect */
		T len_sin_T;  /* length of sin T squared */
		T n_mult;     /* ni over nt */
		T N_dot_V;
		T N_dot_T;

		if ( (N_dot_V = normal.dot(v)) > 0.0 )
			n_mult = ni / nt;
		else
			n_mult = nt / ni;
		cos_V.x = normal.data()[0] * N_dot_V;
		cos_V.y = normal.data()[1] * N_dot_V;
		sin_T.x = (cos_V.x - v.data()[0]) * (n_mult);
		sin_T.y = (cos_V.y - v.data()[1]) * (n_mult);
		if ( (len_sin_T = sin_T.dot(sin_T)) >= 1.0 )
			return false;    // internal reflection
		N_dot_T = sqrt(1.0 - len_sin_T);
		if ( N_dot_V < 0.0 )
			N_dot_T = -N_dot_T;
		result.data()[0] = sin_T.x - normal.data()[0] * N_dot_T;
		result.data()[1] = sin_T.y - normal.data()[1] * N_dot_T;
		return true;
	}

	T x, y;
};

template <typename T> class _Vector3D : public _Vector<T, 3>, protected _DataContainerBase<T> {
	virtual T* _data() { return &x; }
	virtual const T* _data() const { return &x; }
    virtual _DataContainerBase<T> * _clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerStatic<T, 3>();
        tmp->_copy(&this->x, 3);
        return tmp;
    }
public:
	_Vector3D() : _Vector<T, 3>((_DataContainerBase<T>&)*this) {}
	_Vector3D(T c) : _Vector<T, 3>((_DataContainerBase<T>&)*this) {
		this->_fill(c, 3);
	}
	_Vector3D(T x, T y, T z) : _Vector<T, 3>((_DataContainerBase<T>&)*this) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	_Vector3D(const T * arr) : _Vector<T, 3>((_DataContainerBase<T>&)*this) {
		this->_copy(arr, 3);
	}
	_Vector3D(const _Vector<T, 3> & v) : _Vector<T, 3>((_DataContainerBase<T>&)*this) {
		this->_copy(v.container, 3);
	}
	_Vector3D(const _Vector3D<T> & v) : _Vector<T, 3>((_DataContainerBase<T>&)*this) {
		this->_copy(v.container, 3);
	}

	_Vector3D<T> operator + (const _Vector<T, 3> & v) const {
		return _Vector3D<T>(x + v.data()[0], y + v.data()[1], z + v.data()[2]);
	}
	_Vector3D<T> operator - (const _Vector<T, 3> & v) const {
		return _Vector3D<T>(x - v.data()[0], y - v.data()[1], z - v.data()[2]);
	}
	_Vector3D<T> operator * (T c) const {
		return _Vector3D<T>(x * c, y * c, z * c);
	}
	_Vector3D<T> operator / (T c) const {
		return _Vector3D<T>(x / c, y / c, z / c);
	}

	_Vector3D<T> cross(const _Vector<T, 3>& b) const
	{
 		return _Vector3D<T>(y * b.data()[2] - z * b.data()[1],
			z * b.data()[0] - x * b.data()[2],
			x * b.data()[1] - y * b.data()[0]);
	}

	static bool refraction(const _Vector<T, 3>& normal, const _Vector<T, 3>& v, T ni, T nt, _Vector3D<T> & result)
	{
		_Vector3D<T> sin_T;      /* sin vect of the refracted vect */
		_Vector3D<T> cos_V;      /* cos vect of the incident vect */
		T len_sin_T;  /* length of sin T squared */
		T n_mult;     /* ni over nt */
		T N_dot_V;
		T N_dot_T;

	    if ( (N_dot_V = normal.dot(v)) > 0.0 )
			n_mult = ni / nt;
		else
			n_mult = nt / ni;
		cos_V.x = normal.data()[0] * N_dot_V;
		cos_V.y = normal.data()[1] * N_dot_V;
		cos_V.z = normal.data()[2] * N_dot_V;
		sin_T.x = (cos_V.x - v.data()[0]) * (n_mult);
		sin_T.y = (cos_V.y - v.data()[1]) * (n_mult);
		sin_T.z = (cos_V.z - v.data()[2]) * (n_mult);
		if ( (len_sin_T = sin_T.dot(sin_T)) >= 1.0 )
			return false;    // internal reflection
		N_dot_T = sqrt(1.0 - len_sin_T);
		if ( N_dot_V < 0.0 )
			N_dot_T = -N_dot_T;
		result.data()[0] = sin_T.x - normal.data()[0] * N_dot_T;
		result.data()[1] = sin_T.y - normal.data()[1] * N_dot_T;
		result.data()[2] = sin_T.z - normal.data()[2] * N_dot_T;
	
		return true;
	}

	T x, y, z;
};

template <typename T> class _Vector4D : public _Vector<T, 4>, protected _DataContainerBase<T> {
	virtual T* _data() { return &x; }
	virtual const T* _data() const { return &x; }
    virtual _DataContainerBase<T> * _clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerStatic<T, 3>();
        tmp->_copy(&this->x, 4);
        return tmp;
    }
public:
	_Vector4D() : _Vector<T, 4>((_DataContainerBase<T>&)*this) {}
	_Vector4D(T c) : _Vector<T, 4>((_DataContainerBase<T>&)*this) {
		this->_fill(c, 4);
	}
	_Vector4D(T x, T y, T z, T w) : _Vector<T, 4>((_DataContainerBase<T>&)*this) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}
	_Vector4D(const T * arr) : _Vector<T, 4>((_DataContainerBase<T>&)*this) {
		this->_copy(arr, 4);
	}
	_Vector4D(const _Vector<T, 4> & v) : _Vector<T, 4>((_DataContainerBase<T>&)*this) {
		this->_copy(v.container, 4);
	}
	_Vector4D(const _Vector4D<T> & v) : _Vector<T, 4>((_DataContainerBase<T>&)*this) {
		this->_copy(v.container, 4);
	}
	_Vector4D(const _Vector<T, 3> & v) : _Vector<T, 4>((_DataContainerBase<T>&)*this) {
		this->_copy(v.container, 3);
		this->w = 1;
	}

	_Vector4D<T> operator + (const _Vector<T, 4> & v) const {
		return _Vector4D<T>(x + v.data()[0], y + v.data()[1], z + v.data()[2], w + v.data()[3]);
	}
	_Vector4D<T> operator - (const _Vector<T, 4> & v) const {
		return _Vector4D<T>(x - v.data()[0], y - v.data()[1], z - v.data()[2], w - v.data()[3]);
	}
	_Vector4D<T> operator * (T c) const {
		return _Vector4D<T>(x * c, y * c, z * c, w * c);
	}
	_Vector4D<T> operator / (T c) const {
		return _Vector4D<T>(x / c, y / c, z / c, w / c);
	}

	_Vector3D<T> vector3d() const {
		return _Vector3D<T>(x, y, z);
	}

	T x, y, z, w;
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
