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

namespace Locomotion {

#ifndef __AVR__
typedef double real_t;
#else
typedef float real_t;
#endif


//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float FAST_ISQRT(float x){
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Macros for Modern Matrix Methods ("Numerical Recipies in C")
inline int IMIN(int a, int b) {
    return a < b ? a : b;
}
inline real_t FMAX(real_t a, real_t b) {
    return a > b ? a : b;
}
inline real_t SIGN(real_t a, real_t b) {
    return b >= 0.0 ? fabs(a) : -fabs(a);
}
inline real_t SQR(real_t a) {
    return a * a;
}


template<typename T> class _DataContainerBase {
public:
    virtual ~_DataContainerBase() {}

	virtual T* _data() = 0;
	virtual const T* _data() const = 0;

    void fill(T c, size_t N) {
		T * p = this->_data();
		for (size_t i = 0; i < N; i++)
			*(p++) = c;
	}
	void copy(const T * data, size_t N) {
		memcpy(this->_data(), data, sizeof(T) * N);
	}
	virtual void copy(const _DataContainerBase<T> & c, size_t N) {
		copy(c._data(), N);
	}
    virtual _DataContainerBase<T> * clone() const = 0;
};

template<typename T, size_t SIZE> class _DataContainerStatic : public _DataContainerBase<T> {
    T arr[SIZE];
public:
	virtual T* _data() { return arr; }
	virtual const T* _data() const { return arr; }
    virtual _DataContainerBase<T> * clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerStatic<T, SIZE>();
        tmp->copy(this->arr, SIZE);
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
    virtual _DataContainerBase<T> * clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerDynamic<T, SIZE>();
        tmp->copy(this->ptr, SIZE);
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
    virtual _DataContainerBase<T> * clone() const {
        _DataContainerBase<T> * tmp = new _DataContainerFromPtr<T, SIZE, owns>(new T[SIZE]);
        tmp->copy(this->ptr, SIZE);
        return tmp;
    }
};

};

#endif
