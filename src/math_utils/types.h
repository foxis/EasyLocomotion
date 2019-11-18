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

#if !defined(MATH_UTILS_TYPES_H)
#define MATH_UTILS_TYPES_H

#include <math.h>
#include <limits>

namespace Locomotion {

#if !defined(REAL_T)
#ifndef __AVR__
typedef double real_t;
#else
typedef float real_t;
#endif
#else
typedef REAL_T real_t;
#endif

#if !defined(TIMESTAMP_T)
typedef uint32_t timestamp_t;
#else
typedef TIMESTAMP_T timestamp_t;
#endif

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float FAST_ISQRT(float x){
    float halfx = 0.5f * x;
    union {
        float f;
        uint32_t i;
    } tmp;
    tmp.f = x;
    tmp.i = 0x5f3759df - (tmp.i >> 1);
    tmp.f = tmp.f * (1.5f - (halfx * tmp.f * tmp.f));
    tmp.f = tmp.f * (1.5f - (halfx * tmp.f * tmp.f));
    return tmp.f;
}

// Macros for Modern Matrix Methods ("Numerical Recipies in C")
template<typename T> T MIN(T a, T b) { return a < b ? a : b; }
template<typename T> T MAX(T a, T b) { return a > b ? a : b; }
template<typename T> T SIGN(T a, T b) { return b >= 0.0 ? fabs(a) : -fabs(a); }
template<typename T> T SQR(T a) { return a * a; }
template<typename T> void SWAP(T * a, T * b) {
    T tmp = *a;
    *a = *b;
    *b = tmp;
}
template<typename T, size_t M> void SWAP_ROWS(T * a, T * b) {
    T tmp[M];
    memcpy(tmp, a, sizeof(T) * M);
    memcpy(a, b, sizeof(T) * M);
    memcpy(b, tmp, sizeof(T) * M);
}
template<typename T, size_t M> void SWAP_COLS(T * a, T * b) {
    for (size_t i = 0; i < M; i++) {
        SWAP(a, b);
        a += M;
        b += M;
    }
}
template<typename T> T MEAN(T a, T b) { return 0.5 * (a + b); }
template<typename T, size_t N> T SUM_ROW(const T * a) { 
    T acc = 0;
    for (size_t i = 0; i < N; i++) {
        acc += *a;
        ++a;
    }
    return acc;
}
template<typename T, size_t M> T SUM_COL(const T * a) { 
    T acc = 0;
    for (size_t i = 0; i < M; i++) {
        acc += *a;
        a += M;
    }
    return acc; 
}
template<typename T, size_t N> T MEAN_ROW(const T * a) { return SUM_ROW<N>(a) / (T)N; }
template<typename T, size_t M> T MEAN_COL(const T * a) { return SUM_COL<M>(a) / (T)M; }
template<typename T, size_t N> T MAX_ROW(const T * a) { 
    T m = std::numeric_limits<T>::min();
    for (size_t i = 0; i < N; i++) {
        m = MAX(*a, m);
        ++a;
    }
    return m;
}
template<typename T, size_t M> T MAX_COL(const T * a) { 
    T m = std::numeric_limits<T>::min();
    for (size_t i = 0; i < M; i++) {
        m = MAX(*a, m);
        a += M;
    }
    return m; 
}
template<typename T, size_t N> T MIN_ROW(const T * a) { 
    T m = std::numeric_limits<T>::max();
    for (size_t i = 0; i < N; i++) {
        m = MIN(*a, m);
        ++a;
    }
    return m;
}
template<typename T, size_t M> T MIN_COL(const T * a) { 
    T m = std::numeric_limits<T>::max();
    for (size_t i = 0; i < M; i++) {
        m = MIN(*a, m);
        a += M;
    }
    return m;
}
template<typename T, size_t N> void SET_ROW(T * a, T c) { 
    for (size_t i = 0; i < N; i++) {
        *a = c;
        ++a;
    }
}
template<typename T, size_t M, size_t K=M> void SET_COL(T * a, T c) { 
    for (size_t i = 0; i < K; i++) {
        *a = c;
        a += M;
    }
}
template<typename T, size_t M> void SET_ROW(T * dst, const T * src) { 
    for (size_t i = 0; i < M; i++) {
        *dst = *src;
        ++dst;
        ++src;
    }
}
template<typename T, size_t M, size_t K=M> void SET_COL(T * dst, const T * src) { 
    for (size_t i = 0; i < K; i++) {
        *dst = *src;
        dst += M;
        ++src;
    }
}
template<typename T, size_t N> void GET_ROW(T * dst, const T * src) { 
    SET_ROW<T, N>(dst, src);
}
template<typename T, size_t M, size_t K=M> void GET_COL(T * dst, const T * src) { 
    for (size_t i = 0; i < K; i++) {
        *dst = *src;
        src += M;
        ++dst;
    }
}


template<typename T> class _DataContainerBase {
public:
    virtual ~_DataContainerBase() {}

	virtual T* _data() = 0;
	virtual const T* _data() const = 0;

    void _fill(T c, size_t N) {
		T * p = _data();
		for (size_t i = 0; i < N; i++)
			*(p++) = c;
	}
	virtual void _copy(const T * arr, size_t N) {
		memcpy(_data(), arr, sizeof(T) * N);
	}
	virtual void _copy(const _DataContainerBase<T> & c, size_t N) {
		_copy(c._data(), N);
	}
    virtual _DataContainerBase<T> * _clone() const = 0;
};

template<typename T, size_t SIZE> class _DataContainerStatic : public _DataContainerBase<T> {
    T arr[SIZE];
public:
	virtual T* _data() { return arr; }
	virtual const T* _data() const { return arr; }
    virtual _DataContainerBase<T> * _clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerStatic<T, SIZE>();
        tmp->_copy(this->arr, SIZE);
        return tmp;
    }
};

template<typename T, size_t SIZE> class _DataContainerDynamic : public _DataContainerBase<T> {
    T *ptr;
public:
    _DataContainerDynamic() { 
        ptr = new T[SIZE];
    }
    virtual ~_DataContainerDynamic() { 
        delete[] ptr;
    }
	virtual T* _data() { return ptr; }
	virtual const T* _data() const { return ptr; }
    virtual _DataContainerBase<T> * _clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerDynamic<T, SIZE>();
        tmp->_copy(this->ptr, SIZE);
        return tmp;
    }
};

template<typename T, size_t SIZE, bool owns> class _DataContainerFromPtr : public _DataContainerBase<T> {
    T *ptr;
public:
    _DataContainerFromPtr(T * ptr) {
        this->ptr = ptr;
    }
    ~_DataContainerFromPtr() {
        if (owns)
            delete[] ptr;
    }
	virtual T* _data() { return ptr; }
	virtual const T* _data() const { return ptr; }
    virtual _DataContainerBase<T> * _clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerFromPtr<T, SIZE, owns>(new T[SIZE]);
        tmp->_copy(this->ptr, SIZE);
        return tmp;
    }
};

typedef _DataContainerBase<real_t> DataContainerBase;
template<size_t SIZE> using DataContainerStatic = _DataContainerStatic<real_t, SIZE>;
template<size_t SIZE> using DataContainerDynamic = _DataContainerDynamic<real_t, SIZE>;
template<size_t SIZE, bool OWNS=false> using DataContainerFromPtr = _DataContainerFromPtr<real_t, SIZE, OWNS>;

};

#endif
