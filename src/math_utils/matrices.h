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

#if !defined(MATH_UTILS_MATRICES_H)
#define MATH_UTILS_MATRICES_H

#include "vectors.h"

namespace Locomotion {

/// m ------>                     n
/// |        col0 col1 col2... |  |
/// | row0 | 00   10   20...   |  v
/// | row1 | 01   11   21...   |  
/// ...
///
template<typename T, size_t N, size_t M> class _Matrix {
protected:
	_DataContainerBase<T> &container;
public:
	_Matrix(_DataContainerBase<T> &container) : container(container) {}
	_Matrix(_DataContainerBase<T> &container, T c) : _Matrix(container) {
		this->container._fill(c, N * M);
	}
	_Matrix(_DataContainerBase<T> &container, const T * data) : _Matrix(container) {
		this->container._copy(data, N * M);
	}
	_Matrix(_DataContainerBase<T> &container, const _Matrix<T, N, M> & m) : _Matrix(container) {
		this->container._copy(m.container, N * M);
	}

    void eye() {
        eye(*this);
    }
    static void eye(_Matrix<T, N, M> & m) {
        memset(m.data(), 0, sizeof(T) * M * N);
        T *p = m.data();
        for (size_t i = 0; i < min(M, N); i++) {
            *p = 1;
            p += M + 1;
        }
    }

	void operator = (const _Matrix<T, N, M> & m) {
		this->container._copy(m.container, N * M);
	}

    void operator += (const _Matrix<T, N, M> & m) {
        add(m);
    }
    void operator -= (const _Matrix<T, N, M> & m) {
        sub(m);
    }
    void operator *= (T c) {
        mul(c);
    }
    void operator /= (T c) {
        div(c);
    }

    void add(const _Matrix<T, N, M> & m) {
        add(m, *this);
    }
    void add(const _Matrix<T, N, M> & m, _Matrix<T, N, M> & dst) const {
        add(*this, m, dst);
    }
    static void add(const _Matrix<T, N, M> & src, const _Matrix<T, N, M> & m, _Matrix<T, N, M> & dst) {
        const T *p = src.data();
        const T *B = m.data();
        T *pd = dst.data();
        for (size_t i = 0; i < M * N; i++) {
            *pd = *p + *B;
            ++p;
            ++B;
            ++pd;
        }
    }

    void sub(const _Matrix<T, N, M> & m) {
        sub(m, *this);
    }
    void sub(const _Matrix<T, N, M> & m, _Matrix<T, N, M> & dst) const {
        sub(*this, m, dst);
    }
    static void sub(const _Matrix<T, N, M> & src, const _Matrix<T, N, M> & m, _Matrix<T, N, M> & dst) {
        const T *p = src.data();
        const T *B = m.data();
        T *pd = dst.data();
        for (size_t i = 0; i < M * N; i++) {
            *pd = *p - *B;
            ++p;
            ++B;
            ++pd;
        }
    }

    void mul(T c) {
        T *p = data();
        for (size_t i = 0; i < N * M; i++) {
            *p *= c;
            ++p;
        }
    }

    void mul(T c, _Matrix<T, N, M> & dst) const {
        mul(c, *this, dst);
    }

    static void mul(T c, const _Matrix<T, N, M> & src, _Matrix<T, N, M> & dst) {
        const T *p = src.data();
        T *pd = dst.data();
        for (size_t i = 0; i < N * M; i++) {
            *pd = *p * c;
            ++pd;
            ++p;
        }
    }

    void div(T c) {
        T *p = data();
        for (size_t i = 0; i < N * M; i++) {
            *p /= c;
            ++p;
        }
    }

    void div(T c, _Matrix<T, N, M> & dst) const {
        div(c, *this, dst);
    }

    static void div(T c, const _Matrix<T, N, M> & src, _Matrix<T, N, M> & dst) {
        const T *p = src.data();
        T *pd = dst.data();
        for (size_t i = 0; i < N * M; i++) {
            *pd = *p / c;
            ++pd;
            ++p;
        }
    }

    template<size_t K>
    void mul_mat(const _Matrix<T, N, K> & m, _Matrix<T, M, K> & dst) const {
        T *pd = dst.data();
		
        for (size_t j = 0; j < N; j++) {
            for (size_t i = 0; i < K; i++) {
                T acc = 0;
                const T *A = data() + j * M;
                const T *B = m.data() + i;
                for (size_t m = 0; m < M; m++) {
                    acc += *A * *B;
                    ++A;
                    B += K;
                }
                *pd = acc;
                ++pd;
            }
        }
    }

    void mul(const _Vector<T, N> & v, _Vector<T, M> & dst) const {
        T *pd = dst.data();
        const T *src = data();
        const T *pv;
		for (size_t i = 0; i < N; i++) {
			T acc = 0;
            pv = v.data();
			for (size_t j = 0; j < M; j++) {
				acc += *pv * *src;
				++pv;
				++src;
			}
			*pd = acc;
			++pd;
		}
    }

    void mulT(const _Vector<T, M> & v, _Vector<T, N> & dst) const {
        T *pd = dst.data();
        const T *pv;
		for (size_t j = 0; j < M; j++) {
			T acc = 0;
            pv = v.data();
            const T *src = data() + j;
			for (size_t i = 0; i < N; i++) {
				acc += *pv * *src;
				++v;
				src += M;
			}
			*pd = acc;
			++pd;
		}
    }

    void transpose(_Matrix<T, M, N> & m) const {
        T *dst = m.data();
        const T *src = data();
        for (size_t i = 0; i < N; i++) {
            T * p = dst + i;
            for (size_t j = 0; j < M; j++) {
                *p = *src;
                ++src;
                p += M;
            }
        }
    }
    void transpose() {
        static_assert(N == M, "Must be square matrix");
        for (size_t i = 0; i < N - 1; i++) {
            for (size_t j = i + 1; j < N; j++) {
                T tmp = val(i, j);
                *data(i, j) = val(j, i);
                *data(j, i) = tmp;
            }
        }
    }

    ///
    /// Determinant only valid for square matrix
    /// 
    virtual T det() const {
        static_assert(N == M, "Must be square matrix");
        _DataContainerBase<T> * tmp_cont = container._clone();
        _Matrix<T, N, N> tmp(*tmp_cont);

        T ratio, det = 1;
        size_t i, j, k;
        T * pd = tmp.data();
        // upper triangular
        for(i = 0; i < N; i++) {
            for(j = 0; j < N; j++) {
                if(j > i) {
                    ratio = tmp.val(j, i) / tmp.val(i, i);
                    for(k = 0; k < N; k++){
                        *tmp.data(j, k) -= ratio * tmp.val(i, k);
                    }
                }
            }
        }
        for(i = 0; i < N; i++) {
            det *= *pd;
            pd += M + 1;
        }
        delete tmp_cont;
        return det;
    }

    ///
    /// Inverse only valid for square matrix
    /// 
    virtual bool inverse(_Matrix<T, N, N> & dst) const {
        static_assert(N == M, "Must be square matrix");
        // Not implemented
        return false;
    }

    // SVD 'Singular Value Decomposition'
    // W. H. Press, S. A. Teukolsky, W. T. Vetterling, B. P. Flannery
    // 'Numerical Recipes in C'
    // A = UWVT
    // U 
    // W - diagonal matrix (vector of diagonal elements)
    // Rv1 is a vector to store temporary values
    void svd(_Matrix<T, N, M> & U, _Vector<T, M> & W, _Matrix<T, M, M> & V, _Vector<T, M> & Rv1) const {
        bool flag;
        size_t i, its, j, jj, k, l, nm;
        T anorm, c, f, g, h, s, scale, x, y, z;
        U = *this;

        g = scale = anorm = 0.0;  // Householder reduction to bidiagonal form
        for(i = 0; i < M; i++)
        {
            l = i + 1;
            Rv1.data()[i] = scale * g;
            g = s = scale = 0.0;
            if(i < N)
            {
                for(k = i; k < N; k++) 
                    scale += fabs(U.val(k, i));

                if(scale)
                {
                    for(k = i; k < N; k++)
                    {
                        *U.data(k, i) /= scale;
                        s += U.val(k, i) * U.val(k, i);
                    }
                    f = U.val(i, i);
                    g = -SIGN(sqrt(s), f);
                    h = f * g - s;
                    *U.data(i, i) = f - g;
                    for(j = l; j < M; j++)
                    {
                        for(s = 0.0, k = i; k < N; k++) 
                            s += U.val(k, i) * U.val(k, j);
                        f = s / h;
                        for(k = i; k < N; k++) 
                            *U.data(k, j) += f * U.val(k, i);
                    }
                    for(k = i; k < N; k++) 
                        *U.data(k, i) *= scale;
                }
            }
            W.data()[i] = scale * g;
            g = s = scale = 0.0;
            if(i < N && i != (M - 1))
            {
                for(k = l; k < M; k++) 
                    scale += fabs(U.val(i, k));
                if(scale)
                {
                    for(k = l; k < M; k++)
                    {
                        *U.data(i, k) /= scale;
                        s += U.val(i, k) * U.val(i, k);
                    }
                    f = U.val(i, l);
                    g = -SIGN(sqrt(s), f);
                    h = f * g - s;
                    *U.data(i, l) = f - g;
                    for(k = l; k < M; k++) 
                        Rv1.data()[k] = U.val(i, k) / h;

                    for(j = l; j < N; j++)
                    {
                        for(s = 0.0, k = l; k < M; k++) 
                            s += U.val(j, k) * U.val(i, k);
                        for(k = l; k < M; k++) 
                            *U.data(j, k) += s * Rv1.data()[k];
                    }
                    for(k = l; k < M; k++) 
                        *U.data(i, k) *= scale;
                }
            }
            anorm = FMAX(anorm, (fabs(W.data()[i]) + fabs(Rv1.data()[i])));
        }

        for(i = (M - 1); i >= 0; i--)
        { // Accumulation of right-hand transformation
            if(i < (M - 1))
            {
                if(g)
                {
                    for(j = l; j < M; j++)
                        *V.data(j, i) = (U.val(i, j) / U.val(i, l)) / g;
                    for(j = l; j < M; j++)
                    {
                        for(s = 0.0, k = l; k < M; k++) 
                            s += U.val(i, k) * V.val(k, j);
                        for(k = l; k < M; k++) 
                            *V.data(k, j) += s * V.val(k, i);
                    }
                }
                for(j = l; j < M; j++) 
                    *V.data(i, j) = *V.data(j, i) = 0.0;
            }
            *V.data(i, i) = 1.0;
            g = Rv1.data()[i];
            l = i;
        }

        for(i = (IMIN(N, M) - 1); i >= 0; i--)
        {    // Accumulation of left-hand
            l = i + 1;                                 // transformations
            g = W.data()[i];
            for(j = l; j < M; j++) 
                *U.data(i, j) = 0.0;
            if(g)
            {
                g = 1.0 / g;
                for(j = l; j < M; j++)
                {
                    for(s = 0.0, k = l; k < N; k++) 
                        s += U.val(k, i) * U.val(k, j);
                    f = (s / U.val(i, i)) * g;
                    for(k = i; k < N; k++) 
                        *U.data(k, j) += f * U.val(k, i);
                }
                for(j = i; j < N; j++) 
                    *U.data(j, i) *= g;
            }
            else 
                for(j = i; j < N; j++) 
                    *U.data(j, i) = 0.0;
            ++(*U.data(i, i));
        }

        for(k = M - 1; k >= 0; k--)
        {      // Diagonalization of the bidiagonal form: Loop over
            for(its = 1; its <= 30; its++)
            { // singular values, and over allowed iterations
                flag = true;
                for(l = k; l >= 0; l--)
                { // Test for splitting
                    nm = l - 1;
                    if((T)(fabs(Rv1.data()[l]) + anorm) == anorm)
                    {
                        flag = false;
                        break;
                    }
                    if((T)(fabs(W.data()[nm]) + anorm) == anorm) 
                        break;
                }
                if(flag)
                {
                    c = 0.0;  //  Cancellation of Rv1.x[l], if l > 0
                    s = 1.0;
                    for(i = l; i <= k; i++)
                    {
                        f = s * Rv1.data()[i];
                        Rv1.data()[i] = c * Rv1.data()[i];
                        if((T)(fabs(f) + anorm) == anorm) break;
                        g = W.data()[i];
                        h = pytag(f, g);
                        W.data()[i] = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = -f * h;
                        for(j = 0; j < N; j++)
                        {
                            y = U.val(j, nm);
                            z = U.val(j, i);
                            *U.data(j, nm) = y * c + z * s;
                            *U.data(j, i) = z * c - y * s;
                        }
                    }
                }
                z = W.data()[k];
                if(l == k)
                {       // Convergence
                    if(z < 0.0)
                    {  // Singular value is made nonnegative
                        W.data()[k] = -z;
                        for(j = 0; j < M; j++) 
                            *V.data(j, k) = -V.val(j, k);
                    }
                    break;
                }
                if(its == 50) 
                    return false;

                x = W.data()[l];
                nm = k - 1;
                y = W.data()[nm];
                g = Rv1.data()[nm];
                h = Rv1.data()[k];
                f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
                g = pytag(f, 1.0);
                f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
                c = s = 1.0;            // Next QR transformation
                for(j = l; j < nm + 1; j++)
                {
                    i = j + 1;
                    g = Rv1.data()[i];
                    y = W.data()[i];
                    h = s * g;
                    g = c * g;
                    z = pytag(f, h);
                    Rv1.data()[j] = z;
                    c = f / z;
                    s = h / z;
                    f = x * c + g * s;
                    g = g * c - x * s;
                    h = y * s;
                    y *= c;
                    for(jj = 0; jj < M; jj++)
                    {
                        x = V.val(jj, j);
                        z = V.val(jj, i);
                        *V.data(jj, j) = x * c + z * s;
                        *V.data(jj, i) = z * c - x * s;
                    }
                    z = pytag(f, h);
                    W.data()[j] = z;   // Rotaion can be arbitrary if z = 0
                    if(z)
                    {
                        z = 1.0 / z;
                        c = f * z;
                        s = h * z;
                    }
                    f = c * g + s * y;
                    x = c * y - s * g;
                    for(jj = 0; jj < N; jj++)
                    {
                        y = U.val(jj, j);
                        z = U.val(jj, i);
                        *U.data(jj, j) = y * c + z * s;
                        *U.data(jj, i) = z * c - y * s;
                    }
                }
                Rv1.data()[l] = 0.0;
                Rv1.data()[k] = f;
                W.data()[k] = x;
            }
        }
        return true;
    }

    ///
    /// LU decomposition into two triangular matrices
    /// NOTE: Assume, that l&u matrices are set to zero
    void lu(_Matrix<T, N, N> & l, _Matrix<T, N, N> & u) const {
        size_t i, j, k;
        static_assert(N == M, "Must be square matrix");
        for (i = 0; i < N; i++) {
            for (j = 0; j < N; j++) {
                if (j >= i) {
                    *l.data(j, i) = this->val(j, i);
                    for (k = 0; k < i; k++) {
                        *l.data(j, i) = l.val(j, i) - l.val(j, k) * u.val(k, i);
                    }
                } else {
                    *l.data(j, i) = 0;
                }
            }
            for (j = 0; j < N; j++) {
                if (j < i)
                    *u.data(i, j) = 0;
                else if (j == i)
                    *u.data(i, j) = 1;
                else {
                    *u.data(i, j) = this->val(i, j) / l.val(i, i);
                    for (k = 0; k < i; k++)
                        *u.data(i, j) = u.val(i, j) - ((l.val(i, k) * u.val(k, j)) / l.val(i, i));
                }
            }
        }
    }

private:

    // (a^2+b^2)^(1/2) without Owerflow
    inline T pytag(T a, T b) const
    {
        const T absa = fabs(a), absb = fabs(b);
        if(absa > absb) 
            return absa * sqrt(1.0 + SQR(absb / absa));
        else 
            return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + SQR(absa / absb)));
    }

public:
	inline T* data() { return container._data(); }
	inline const T* data() const { return container._data(); }

	inline T val(size_t row, size_t col) const { return *data(row, col); }
	inline T* data(size_t row, size_t col) { return this->row(row) + col; }
	inline const T* data(size_t row, size_t col) const { return this->row(row) + col; }
	inline T* row(size_t row) { return data() + row * M; }
	inline const T* row(size_t row) const { return data() + row * M; }
};

template <class T> class _Matrix2x2 : public _Matrix<T, 2, 2>, private _DataContainerStatic<T, 4> {
public:
	_Matrix2x2() : _Matrix<T, 2, 2>((_DataContainerBase<T>&)*this) {}
	_Matrix2x2(T c) : _Matrix<T, 2, 2>((_DataContainerBase<T>&)*this) {
        this->_fill(c, 4);
    }
	_Matrix2x2(T a, T b, T c, T d) : _Matrix<T, 2, 2>((_DataContainerBase<T>&)*this) {
        T * p = this->data(); 
		*(p++) = a;
		*(p++) = b;
		*(p++) = c;
		*(p++) = d;
	}
	_Matrix2x2(const T * data) : _Matrix<T, 2, 2>((_DataContainerBase<T>&)*this) {
        this->_copy(data, 4);
    }
	_Matrix2x2(const _Matrix<T, 2, 2> & m) : _Matrix<T, 2, 2>((_DataContainerBase<T>&)*this) {
        this->_copy(m.container, 4);
    }
	_Matrix2x2(const _Matrix2x2<T> & m) : _Matrix<T, 2, 2>((_DataContainerBase<T>&)*this) {
        this->_copy(m.container, 4);
    }

    _Matrix2x2 operator + (const _Matrix<T, 2, 2> & m) const {
        _Matrix2x2 tmp(*this);
        tmp += m;
        return tmp;
    }
    _Matrix2x2 operator - (const _Matrix<T, 2, 2> & m) const {
        _Matrix2x2 tmp(*this);
        tmp -= m;
        return tmp;
    }
    _Matrix2x2 operator * (const _Matrix<T, 2, 2> & m) const {
        _Matrix2x2 tmp;
        this->mul_mat(m, tmp);
        return tmp;
    }
    _Vector2D<T> operator * (const _Vector<T, 2> & v) const {
        _Vector2D<T> tmp;
        this->mul(v, tmp);
        return tmp;
    }
    _Matrix2x2 operator * (const T c) const {
        _Matrix2x2 tmp(*this);
        tmp *= c;
        return tmp;
    }
    _Matrix2x2 operator / (const T c) const {
        _Matrix2x2 tmp(*this);
        tmp /= c;
        return tmp;
    }

	virtual T det() const {
        const T * p = this->data(); 
        return p[0] * p[3] - p[1] * p[2];
    }

    virtual bool inverse(_Matrix2x2<T> & dst) const {
        const T * p = this->data();
        T * p1 = dst.data();
        T D = det();
        T a = *(p++), b = *(p++), c = *(p++), d = *(p++);
        if (D == 0) 
            return false;
        *(p1++) = d / D;
        *(p1++) = -b / D;
        *(p1++) = -c / D;
        *(p1++) = a / D;
        return true;
    }

    void rotation(T theta) {
        rotation(theta, *this);
    }

    void rotation(T theta, _Matrix<T, 2, 2> & dst) const {
        T *p = dst.data();
        T c = cos(theta);
        T s = sin(theta);
        *(p++) = c;
        *(p++) = -s;
        *(p++) = s;
        *(p++) = c;
    } 
};

template <class T> class _Matrix3x3 : public _Matrix<T, 3, 3>, private _DataContainerStatic<T, 9> {
public:
	_Matrix3x3() : _Matrix<T, 3, 3>((_DataContainerBase<T>&)*this) {}
	_Matrix3x3(T c) : _Matrix<T, 3, 3>((_DataContainerBase<T>&)*this) {
        this->_fill(c, 9);
    }
	_Matrix3x3(T a, T b, T c, T d, T e, T f, T g, T h, T i) : _Matrix<T, 3, 3>((_DataContainerBase<T>&)*this) {
        T * p = this->data(); 
		*(p++) = a;
		*(p++) = b;
		*(p++) = c;
		*(p++) = d;
		*(p++) = e;
		*(p++) = f;
		*(p++) = g;
		*(p++) = h;
		*(p++) = i;
	}
	_Matrix3x3(const T * data) : _Matrix<T, 3, 3>((_DataContainerBase<T>&)*this) {
        this->_copy(data, 9);
    }
	_Matrix3x3(const _Matrix<T, 3, 3> & m) : _Matrix<T, 3, 3>((_DataContainerBase<T>&)*this) {
        this->_copy(m.container, 9);
    }
	_Matrix3x3(const _Matrix3x3<T> & m) : _Matrix<T, 3, 3>((_DataContainerBase<T>&)*this) {
        this->_copy(m.container, 9);
    }

    _Matrix3x3 operator + (const _Matrix<T, 3, 3> & m) const {
        _Matrix3x3 tmp(*this);
        tmp += m;
        return tmp;
    }
    _Matrix3x3 operator - (const _Matrix<T, 3, 3> & m) const {
        _Matrix3x3 tmp(*this);
        tmp -= m;
        return tmp;
    }
    _Matrix3x3 operator * (const _Matrix<T, 3, 3> & m) const {
        _Matrix3x3 tmp;
        this->mul_mat(m, tmp);
        return tmp;
    }
    _Vector3D<T> operator * (const _Vector<T, 3> & v) const {
        _Vector3D<T> tmp;
        this->mul(v, tmp);
        return tmp;
    }
    _Matrix3x3 operator * (const T c) const {
        _Matrix3x3 tmp(*this);
        tmp *= c;
        return tmp;
    }
    _Matrix3x3 operator / (const T c) const {
        _Matrix3x3 tmp(*this);
        tmp /= c;
        return tmp;
    }

	virtual T det() const { 
        const T * p = this->data();
        return p[0] * (p[4] * p[8] - p[5] * p[7]) 
            - p[1] * (p[3] * p[8] - p[5] * p[6]) 
            + p[2] * (p[3] * p[7] - p[4] * p[6]);
    }

    virtual bool inverse(_Matrix3x3<T> & dst) const {
        const T * p = this->data();
        T * p1 = dst.data();
        T D = det();
        T a = *(p++), b = *(p++), c = *(p++);
        T d = *(p++), e = *(p++), f = *(p++);
        T g = *(p++), h = *(p++), i = *(p++);
        if (D == 0)
            return false;
        // calculate matrix of cofactors    
        *(p1++) = e * i - h * f;
        *(p1++) = -(d * i - f * g);
        *(p1++) = d * h - e * g;
        *(p1++) = -(b * i - c * h);
        *(p1++) = a * i - c * g;
        *(p1++) = -(a * h - b * g);
        *(p1++) = b * f - c * e;
        *(p1++) = -(a * f - c * d);
        *(p1++) = a * e - b * d;
        dst.transpose();
        dst /= D;
        return true;
    }

    void rotation_x(T theta) {
        rotation_x(theta, *this);
    }
    void rotation_y(T theta) {
        rotation_y(theta, *this);
    }
    void rotation_z(T theta) {
        rotation_z(theta, *this);
    }

    void rotation_x(T theta, _Matrix<T, 3, 3> & dst) const {
        T *p = dst.data();
        T c = cos(theta);
        T s = sin(theta);
        *(p++) = 1;
        *(p++) = 0;
        *(p++) = 0;
        *(p++) = 0;
        *(p++) = c;
        *(p++) = -s;
        *(p++) = 0;
        *(p++) = s;
        *(p++) = c;
    } 
    void rotation_y(T theta, _Matrix<T, 3, 3> & dst) const {
        T *p = dst.data();
        T c = cos(theta);
        T s = sin(theta);
        *(p++) = c;
        *(p++) = 0;
        *(p++) = s;
        *(p++) = 0;
        *(p++) = 1;
        *(p++) = 0;
        *(p++) = -s;
        *(p++) = 0;
        *(p++) = c;
    } 
    void rotation_z(T theta, _Matrix<T, 3, 3> & dst) const {
        T *p = dst.data();
        T c = cos(theta);
        T s = sin(theta);
        *(p++) = c;
        *(p++) = -s;
        *(p++) = 0;
        *(p++) = s;
        *(p++) = c;
        *(p++) = 0;
        *(p++) = 0;
        *(p++) = 0;
        *(p++) = 1;
    } 

    void orientation(const _Vector<T, 3> &axis, T theta) {
        orientation(axis, theta);
    }

    void orientation(const _Vector<T, 3> &axis, T theta, _Matrix<T, 3, 3> & dst) const {
        // calculate attention quaternion
        // NOTE: axis must be unit vector
        theta /= 2.0;
        const T qr = cos(theta);
        const T s = sin(theta);
        const T qi = axis.x * s, qj = axis.y * s, qk = axis.z * s;

        // calculate quaternion rotation matrix
        T * p = dst.data();
        const T qjqj = qj * qj;
        const T qiqi = qi * qi;
        const T qkqk = qk * qk;
        const T qiqj = qi * qj;
        const T qjqr = qj * qr;
        const T qiqk = qi * qk;
        const T qiqr = qi * qr;
        const T qkqr = qk * qr;
        const T qjqk = qj * qk;
        *(p++) = 1.0 - 2.0 * (qjqj + qkqk);
        *(p++) = 2.0 * (qiqj + qkqr);
        *(p++) = 2.0 * (qiqk + qjqr);
        *(p++) = 2.0 * (qiqj + qkqr);
        *(p++) = 1.0 - 2.0 * (qiqi + qkqk);
        *(p++) = 2.0 * (qjqk + qiqr);
        *(p++) = 2.0 * (qiqk + qjqr);
        *(p++) = 2.0 * (qjqk + qiqr);
        *(p++) = 1.0 - 2.0 * (qiqi + qjqj);
    }
};

template <class T> class _Matrix4x4 : public _Matrix<T, 4, 4>, private _DataContainerStatic<T, 16> {
public:
	_Matrix4x4() : _Matrix<T, 4, 4>((_DataContainerBase<T>&)*this) {}
	_Matrix4x4(T c) : _Matrix<T, 4, 4>((_DataContainerBase<T>&)*this) {
        this->_fill(c, 16);
    }
	_Matrix4x4(T a, T b, T c, T d, T e, T f, T g, T h, T i) : _Matrix<T, 4, 4>((_DataContainerBase<T>&)*this) {
        T * p = this->data(); 
		*(p++) = a;
		*(p++) = b;
		*(p++) = c;
		*(p++) = d;
		*(p++) = e;
		*(p++) = f;
		*(p++) = g;
		*(p++) = h;
		*(p++) = i;
	}
	_Matrix4x4(const T * data) : _Matrix<T, 4, 4>((_DataContainerBase<T>&)*this) {
        this->_copy(data, 16);
    }
	_Matrix4x4(const _Matrix<T, 4, 4> & m) : _Matrix<T, 4, 4>((_DataContainerBase<T>&)*this) {
        this->_copy(m.container, 16);
    }
	_Matrix4x4(const _Matrix4x4<T> & m) : _Matrix<T, 4, 4>((_DataContainerBase<T>&)*this) {
        this->_copy(m.container, 16);
    }

    void operator = (const _Matrix3x3<T> & m) {
        const T * sp = m.data();
        T * dp = this->data();
        *(dp++) = *(sp++);
        *(dp++) = *(sp++);
        *(dp++) = *(sp++);
        *(dp++) = 0;
        *(dp++) = *(sp++);
        *(dp++) = *(sp++);
        *(dp++) = *(sp++);
        *(dp++) = 0;
        *(dp++) = *(sp++);
        *(dp++) = *(sp++);
        *(dp++) = *(sp++);
        *(dp++) = 0;
        *(dp++) = 0;
        *(dp++) = 0;
        *(dp++) = 0;
        *(dp++) = 1;
    }

    _Matrix4x4 operator + (const _Matrix<T, 4, 4> & m) const {
        _Matrix4x4 tmp(*this);
        tmp += m;
        return tmp;
    }
    _Matrix4x4 operator - (const _Matrix<T, 4, 4> & m) const {
        _Matrix4x4 tmp(*this);
        tmp -= m;
        return tmp;
    }
    _Matrix4x4 operator * (const _Matrix<T, 4, 4> & m) const {
        _Matrix4x4 tmp;
        this->mul_mat(m, tmp);
        return tmp;
    }
    _Vector4D<T> operator * (const _Vector<T, 4> & v) const {
        _Vector4D<T> tmp;
        this->mul(v, tmp);
        return tmp;
    }
    _Matrix4x4 operator * (const T c) const {
        _Matrix4x4 tmp(*this);
        tmp *= c;
        return tmp;
    }
    _Matrix4x4 operator / (const T c) const {
        _Matrix4x4 tmp(*this);
        tmp /= c;
        return tmp;
    }

	virtual T det() const { 
        const T * P = this->data();
        T a = *(P++), b = *(P++), c = *(P++), d = *(P++);
        T e = *(P++), f = *(P++), g = *(P++), h = *(P++);
        T i = *(P++), j = *(P++), k = *(P++), l = *(P++);
        T m = *(P++), n = *(P++), o = *(P++), p = *(P++);
        T kp = k * p, lo = l * o, jp = j * p, ln = l * n;
        T jo = j * o, kn = k * n, km = k * m, jm = j * m;
        T in = i * n, lm = l * m, ip = i * p, io = i * o;
        return a * (f * (kp - lo) - g * (jp + ln) + h * (jo - kn))
            - b * (e * (kp - lo) - g * (ip + lm) + h * (io - km))
            + c * (e * (jp - ln) - f * (ip + lm) + h * (in - jm))
            - d * (e * (jo - kn) - f * (io + km) + g * (in - jm));
    }

    virtual bool inverse(_Matrix4x4<T> & dst) const {
        T D = det();
        _Matrix3x3<T> tmp;
        if (D == 0)
            return false;
        // calculate matrix of cofactors    
        for (size_t j = 0; j < 4; j++) {
            for (size_t i = 0; i < 4; i++) {
                for (size_t k = 0, kk = 0; k < 4; k++) {
                    if (k != j) {
                        for (size_t l = 0, ll = 0; l < 4; l++) {
                            if (l != i) {
                                *tmp.data(kk, ll) = this->val(k, l);
                                ++ll;
                            }
                        }
                        ++kk;
                    }
                }
                const T c = 1.0 - (2.0 * ((i + j) % 2));
                *dst.data(i, j) = c * tmp.det();
            }
        }
        dst /= D;
        return true;
    }
};

template <class T, size_t N, size_t M> class _MatrixStatic : public _Matrix<T, N, M>, private _DataContainerStatic<T, N * M> {
public:
	_MatrixStatic() : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {}
	_MatrixStatic(T c) : _Matrix<T, N, M>(*this, c) {}
	_MatrixStatic(const T * data) : _Matrix<T, N, M>(*this, data) {}
	_MatrixStatic(const _Matrix<T, N, M> & v) : _Matrix<T, N, M>(*this, v) {}
};

template <class T, size_t N, size_t M> class _MatrixDynamic : public _Matrix<T, N, M>, private _DataContainerDynamic<T, N * M> {
public:
	_MatrixDynamic() : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {}
	_MatrixDynamic(T c) : _Matrix<T, N, M>(*this, c) {}
	_MatrixDynamic(const T * data) : _Matrix<T, N, M>(*this, data) {}
	_MatrixDynamic(const _Matrix<T, N, M> & v) : _Matrix<T, N, M>(*this, v) {}
};


typedef _Matrix2x2<real_t> Matrix2x2;
typedef _Matrix3x3<real_t> Matrix3x3;
typedef _Matrix4x4<real_t> Matrix4x4;

}

#endif
