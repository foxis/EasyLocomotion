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
#include <type_traits>

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
        for (size_t i = 0; i < MIN(M, N); i++) {
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

    template<size_t row=0, size_t col=0, size_t NN, size_t MM>
    void get_submatrix(_Matrix<T, NN, MM> & m) const {
        static_assert(NN <= N && MM <= M, "Target matrix cannot be bigger than the source matrix");
        static_assert(row + NN <= N && col + MM <= M, "Target matrix cannot go outside the source matrix");
        T * dst = m.data();
        for (size_t i = 0; i < NN; i++) {
            const T * src = this->row(i + row) + col;
            for (size_t j = 0; j < MM; j++) {
                *dst = *src;
                ++dst;
                ++src;
            }
        }
    }
    template<size_t row=0, size_t col=0, size_t NN, size_t MM>
    void set_submatrix(const _Matrix<T, NN, MM> & m) {
        static_assert(NN <= N && MM <= M, "Target matrix cannot be bigger than the source matrix");
        static_assert(row + NN <= N && col + MM <= M, "Target matrix cannot go outside the source matrix");
        const T * src = m.data();
        for (size_t i = 0; i < NN; i++) {
            T * dst = this->row(i + row) + col;
            for (size_t j = 0; j < MM; j++) {
                *dst = *src;
                ++dst;
                ++src;
            }
        }
    }

    void get_diagonal(_Vector<T, N> & m) const {
        for (size_t i = 0; i < MIN(M, N); i++)
            m.data()[i] = this->val(i, i);
    }
    void set_diagonal(const _Vector<T, N> & m) {
        for (size_t i = 0; i < MIN(M, N); i++)
            this->row(i)[i] = m.val(i);
    }

    template <typename Dummy = void>
    auto get_diagonal(_Vector<T, M> & m) const -> typename std::enable_if<N != M, Dummy>::type {
        for (size_t i = 0; i < MIN(M, N); i++)
            m.data()[i] = this->val(i, i);
    }
    template <typename Dummy = void>
    auto set_diagonal(const _Vector<T, M> & m) -> typename std::enable_if<N != M, Dummy>::type {
        for (size_t i = 0; i < MIN(M, N); i++)
            this->row(i)[i] = m.val(i);
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
    void mul_mat(const _Matrix<T, M, K> & m, _Matrix<T, N, K> & dst) const {
        for (size_t i = 0; i < N; i++) {
            for (size_t j = 0; j < K; j++) {
                T acc = 0;
                for (size_t k = 0; k < M; k++)
                    acc += this->val(i, k) * m.val(k, j);
                dst.row(i)[j] = acc;
            }
        }
    }

    // multiply this matrix by a diagonal matrix stored in a vector
    template <typename Dummy = void>
    auto mul_mat(const _Vector<T, N> & m, _Matrix<T, N, N> & dst) const  -> typename std::enable_if<N == M, Dummy>::type {
        for(size_t i = 0; i < N; i++)
            for(size_t j = 0; j < N; j++)
                *dst.data(i, j) = val(i, j) * m.data()[j];
    }

    template <typename Dummy = void>
    auto mul_mat(const _Vector<T, N> & m)  -> typename std::enable_if<N == M, Dummy>::type {
        for(size_t i = 0; i < N; i++)
            for(size_t j = 0; j < N; j++)
                *data(i, j) *= m.data()[j];
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
                p += N;
            }
        }
    }

    template <typename Dummy = void>
    auto transpose() -> typename std::enable_if<N == M, Dummy>::type {
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
    /// Undefined behavior for non square matrices
    virtual T det() const {
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
    /// Undefined behavior for non square matrices 
    virtual bool inverse(_Matrix<T, N, N> & dst) const {
        // Not implemented
        return false;
    }

    ///
    /// Moore–Penrose inverse
    /// A = UWVt
    /// Ax = VWxUt
    /// Note: Destroys original matrix
    template <typename Dummy = bool>
    auto pinv(_Matrix<T, M, N> & dst, T eps=1e-9) -> typename std::enable_if<(N <= M), Dummy>::type {
        _DataContainerDynamic<T, N * N> Ucont;
        _DataContainerDynamic<T, N * M> Xcont;
        _DataContainerDynamic<T, N * M> Wxcont;
        _DataContainerDynamic<T, M * N> VWxcont;
        _DataContainerDynamic<T, M * M> Vcont;
        _DataContainerDynamic<T, M> wcont;
        _DataContainerDynamic<T, M> Rcont;

        _Matrix<T, N, N> U(Ucont);
        _Matrix<T, M, M> V(Vcont);
        _Matrix<T, M, N> Wx(Wxcont, 0.0);

        _Vector<T, M> w(wcont);
        _Matrix<T, M, N> X(Xcont);
        _Vector<T, M> Rv(Rcont);

        _Matrix<T, M, N> VWx(VWxcont);

        if (!svd(U, w, V, Rv, X))
            return false;
        for (size_t i = 0; i < M; i++) {
            const T tmp = w.val(i);
            if (fabs(tmp) > eps)
                w.data()[i] = 1.0 / tmp;
            else
                w.data()[i] = 0.0;
        }
        Wx.set_diagonal(w);
        U.transpose();
        V.mul_mat(Wx, VWx);
        VWx.mul_mat(U, dst);
        return true;
    }

    template <typename Dummy = bool>
    auto pinv(_Matrix<T, M, N> & dst, T eps=1e-9) -> typename std::enable_if<(N > M), Dummy>::type {
        _DataContainerStatic<T, N * M> Atcont;
        _Matrix<T, M, N> At(Atcont);
        this->transpose(At);
        if (!At.pinv(*this, eps))
            return false;
        this->transpose(dst);
        return true;
    }

    /// SVD 'Singular Value Decomposition'
    /// W. H. Press, S. A. Teukolsky, W. T. Vetterling, B. P. Flannery
    /// 'Numerical Recipes in C'
    /// A = UWVT
    /// U 
    /// W - diagonal matrix (vector of diagonal elements)
    /// Rv1 is a vector to store temporary values
    /// X is a matrix to store temporary values
    /// Note: Destroys original matrix. Works only on M >= N matrices.
    template <typename Dummy = bool>
    auto svd(_Matrix<T, N, N> & U, _Vector<T, M> & W, _Matrix<T, M, M> & V, _Vector<T, M> & Rv1, _Matrix<T, M, N> & X) -> typename std::enable_if<N <= M, Dummy>::type {
        size_t i, its, j, jj, k, l, nm, _NN;
        bool flag;
        T anorm, c, f, g, h, s, scale, x, y, z;
        T *_x, *_y;
        #define __PARALLEL_N 4
        T p[__PARALLEL_N];

        g = scale = anorm = 0.0;  // Householder reduction to bidiagonal form
        for(i = 0; i < M; i++)
        {
            l = i + 1;
            Rv1.data()[i] = scale * g;
            g = s = scale = 0.0;
            if(i < N)
            {
                for(k = i; k < N; k++) 
                    scale += fabs(this->val(k, i));

                if(scale)
                {
                    for(k = i; k < N; k++)
                        s += SQR((this->row(k)[i] /= scale));
                    f = *(_x = this->row(i) + i);
                    g = -SIGN((T)sqrt(s), f);
                    h = f * g - s;
                    *_x = f - g;
                    for(j = l; j < M; j++)
                    {
                        for(s = 0.0, k = i; k < N; k++) 
                        {
                            _x = this->row(k);
                            s += (_x[i]) * (_x[j]);
                        }
                        f = s / h;
                        for(k = i; k < N; k++) 
                        {
                            _x = this->row(k);
                            _x[j] += f * _x[i];
                        }
                    }
                    for(k = i; k < N; k++) 
                        *this->data(k, i) *= scale;
                }
            }
            W.data()[i] = scale * g;
            g = s = scale = 0.0;
            if(i < N && i != (M - 1))
            {
                _x = this->row(i) + l;
                for(k = l; k < M; k++) 
                    scale += fabs(*(_x++));

                if(scale==0)
                    continue;

                _x = this->row(i) + l;
                for(k = l; k < M; k++)
                    s += SQR((*(_x++) /= scale));

                f = *(_x = this->row(i) + l);
                g = -SIGN((T)sqrt(s), f);
                h = f * g - s;
                *_x = f - g;
                _x = this->row(i) + l;
                _y = Rv1.data() + l;
                for(k = l; k < M; k++) 
                    *(_y++) = *(_x++) / h;

                for(j = l; j < N; j++)
                {
                    _x = this->row(j) + l;
                    _y = this->row(i) + l;
                    _NN = M - l;

                    for(s = 0.0, k = l; k < M; k++) 
                        s += *(_x++) * *(_y++);

                    _x = this->row(j) + l;
                    _y = Rv1.data() + l;
                    for(k = 0; k < (_NN % __PARALLEL_N); k++) 
                        *(_x++) += s * *(_y++);

                    for(; k < _NN; k += __PARALLEL_N) 
                    {
                        *(_x++) += s * *(_y++);
                        *(_x++) += s * *(_y++);
                        *(_x++) += s * *(_y++);
                        *(_x++) += s * *(_y++);
                    }
                }
                _y = this->row(i) + l;
                for(k = l; k < M; k++) 
                    *(_y++) *= scale;
            }
            anorm = MAX(anorm, (T)(fabs(W.val(i)) + fabs(Rv1.val(i))));
        }

        for(i = (M - 1); i >= 0 && i < N * M; i--)
        { // Accumulation of right-hand transformation
            if(i < (M - 1))
            {
                if(g)
                {
                    const T tmp = (this->val(i, l) * g);
                    _x = this->row(i) + l;
                    _y = V.row(i) + l;
                    for(j = l; j < M; j++)
                        *(_y++) = *(_x++) / tmp;

                    for(j = l; j < M; j++)
                    {
                        _x = this->row(i) + l;
                        _y = V.row(j) + l;
                        _NN = M - l;
                        memset(p, 0, sizeof(p));
                        for(k = 0, s=0; k < (_NN % __PARALLEL_N); k++) 
                            s += *(_x++) * *(_y++);
                        
                        for(; k < _NN; k += __PARALLEL_N) 
                        {
                            p[0] += *(_x++) * *(_y++);
                            p[1] += *(_x++) * *(_y++);
                            p[2] += *(_x++) * *(_y++);
                            p[3] += *(_x++) * *(_y++);
                        }
                        s += p[0] + p[1] + p[2] + p[3];
                        _y = V.row(i) + l;
                        _x = V.row(j) + l;
                        for(k = 0; k < (_NN % __PARALLEL_N); k++) 
                            *(_x++) += s * *(_y++);

                        for(; k < _NN; k += __PARALLEL_N) 
                        {
                            *(_x++) += s * *(_y++);
                            *(_x++) += s * *(_y++);
                            *(_x++) += s * *(_y++);
                            *(_x++) += s * *(_y++);
                        }
                    }
                }
                for(j = l; j < M; j++) 
                    *V.data(j, i) = *V.data(i, j) = 0.0;
            }
            *V.data(i, i) = 1.0;
            g = Rv1.data()[i];
            l = i;
        }

        //
        this->transpose(X);

        for(i = (MIN(N, M) - 1); i >= 0 && i < N * M; i--)
        {    // Accumulation of left-hand
            l = i + 1;                                 // transformations
            g = W.val(i);
            for (j = l; j < M; j++)
                *X.data(j, i) = 0;
            if(g)
            {
                g = 1.0 / g;
                for(j = l; j < M; j++)
                {
                    _x = X.row(i) + l;
                    _y = X.row(j) + l;
                    _NN = N - l;
                    for(k = 0; k < (_NN % __PARALLEL_N); k++) 
                        p[k] = *(_x++) * *(_y++);

                    memset(p + k, 0, sizeof(T) * (__PARALLEL_N - k));
                    for(; k < _NN; k += __PARALLEL_N ) 
                    {
                        p[0] += *(_x++) * *(_y++);
                        p[1] += *(_x++) * *(_y++);
                        p[2] += *(_x++) * *(_y++);
                        p[3] += *(_x++) * *(_y++);
                    }
                    f = ((p[0] + p[1] + p[2] + p[3]) / X.val(i, i)) * g;
                    _x = X.row(i) + i;
                    _y = X.row(j) + i;
                    _NN = N - i;
                    for(k = 0; k < (_NN % __PARALLEL_N); k++) 
                        *(_y++) += f * *(_x++);

                    for(; k < _NN; k += __PARALLEL_N ) 
                    {
                        *(_y++) += f * *(_x++);
                        *(_y++) += f * *(_x++);
                        *(_y++) += f * *(_x++);
                        *(_y++) += f * *(_x++);
                    }
                }
                _x = X.row(i) + i;
                for(j = i; j < N; j++) 
                    *(_x++) *= g;
            }
            else 
                memset(X.row(i) + i, 0, sizeof(N - i));
            ++(*X.data(i, i));
        }

        for(k = M - 1; k >= 0 && k < N * M; k--)
        {      // Diagonalization of the bidiagonal form: Loop over
            for(its = 1; its <= 30; its++)
            { // singular values, and over allowed iterations
                flag = true;
                for(l = k; l >= 0 && l < N * M; l--)
                { // Test for splitting
                    nm = l - 1;
                    if((T)(fabs(Rv1.val(l)) + anorm) == anorm)
                    {
                        flag = false;
                        break;
                    }
                    if((T)(fabs(W.val(nm)) + anorm) == anorm) 
                        break;
                }

                if(flag)
                {
                    c = 0.0;  //  Cancellation of Rv1.x[l], if l > 0
                    s = 1.0;
                    for(i = l; i <= k; i++)
                    {
                        f = s * Rv1.val(i);
                        if((T)(fabs(f) + anorm) == anorm) break;
                        Rv1.data()[i] = c * Rv1.val(i);
                        g = W.val(i);
                        h = pytag(f, g);
                        W.data()[i] = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = -f * h;
                        _x = X.row(nm);
                        _y = X.row(i);
                        for(j = 0; j < (N % __PARALLEL_N); j++) 
                        {
                            const T y = *_x;
                            const T z = *_y;
                            *(_x++) = y * c + z * s;
                            *(_y++) = z * c - y * s;
                        }
                        for(; j < N; j += __PARALLEL_N) 
                        {
                            {const T y = *_x;
                            const T z = *_y;
                            *(_x++) = y * c + z * s;
                            *(_y++) = z * c - y * s;}
                            {const T y = *_x;
                            const T z = *_y;
                            *(_x++) = y * c + z * s;
                            *(_y++) = z * c - y * s;}
                            {const T y = *_x;
                            const T z = *_y;
                            *(_x++) = y * c + z * s;
                            *(_y++) = z * c - y * s;}
                            {const T y = *_x;
                            const T z = *_y;
                            *(_x++) = y * c + z * s;
                            *(_y++) = z * c - y * s;}
                        }
                    }
                }
                z = W.data()[k];
                if(l == k)
                {       // Convergence
                    if(z < 0.0)
                    {  // Singular value is made nonnegative
                        W.data()[k] = -z;
                        _x = V.row(k);
                        for(j = 0; j < M; j++) {
                            *_x = -*_x;
                            ++_x;
                        }
                    }
                    break;
                }
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
                    _x = V.row(j);
                    _y = V.row(i);
                    for(jj = 0; jj < (M % __PARALLEL_N); jj++) 
                    {
                        const T x = *_x;
                        const T z = *_y;
                        *(_x++) = x * c + z * s;
                        *(_y++) = z * c - x * s;
                    }
                    for(; jj < M; jj += __PARALLEL_N) 
                    {
                        {const T x = *_x;
                        const T z = *_y;
                        *(_x++) = x * c + z * s;
                        *(_y++) = z * c - x * s;}
                        {const T x = *_x;
                        const T z = *_y;
                        *(_x++) = x * c + z * s;
                        *(_y++) = z * c - x * s;}
                        {const T x = *_x;
                        const T z = *_y;
                        *(_x++) = x * c + z * s;
                        *(_y++) = z * c - x * s;}
                        {const T x = *_x;
                        const T z = *_y;
                        *(_x++) = x * c + z * s;
                        *(_y++) = z * c - x * s;}
                    }
                    z = pytag(f, h);
                    W.data()[j] = z;   // Rotation can be arbitrary if z = 0
                    if(z)
                    {
                        z = 1.0 / z;
                        c = f * z;
                        s = h * z;
                    }
                    f = c * g + s * y;
                    x = c * y - s * g;
                    _x = X.row(j);
                    _y = X.row(i);
                    for(jj = 0; jj < (N % __PARALLEL_N); jj++) 
                    {
                        const T y = *_x;
                        const T z = *_y;
                        *(_x++) = y * c + z * s;
                        *(_y++) = z * c - y * s;
                    }
                    for(; jj < N; jj += __PARALLEL_N) 
                    {
                        {const T y = *_x;
                        const T z = *_y;
                        *(_x++) = y * c + z * s;
                        *(_y++) = z * c - y * s;}
                        {const T y = *_x;
                        const T z = *_y;
                        *(_x++) = y * c + z * s;
                        *(_y++) = z * c - y * s;}
                        {const T y = *_x;
                        const T z = *_y;
                        *(_x++) = y * c + z * s;
                        *(_y++) = z * c - y * s;}
                        {const T y = *_x;
                        const T z = *_y;
                        *(_x++) = y * c + z * s;
                        *(_y++) = z * c - y * s;}
                    }
                }
                Rv1.data()[l] = 0.0;
                Rv1.data()[k] = f;
                W.data()[k] = x;
            }
        }
        if(its >= 30) 
            return false;

        X.transpose(*this);

        V.transpose();
        this->get_submatrix(U);

        return true;
        #undef __PARALLEL_N
    }

    ///
    /// LU decomposition into two triangular matrices
    /// NOTE: Assume, that l&u matrices are set to zero
    template <typename Dummy = void>
    auto lu(_Matrix<T, N, N> & L, _Matrix<T, N, N> & U) const -> typename std::enable_if<N == M, Dummy>::type {
        size_t i, j, k;

        for (i = 0; i < N; i++) { 
            // Upper Triangular 
            for (k = i; k < N; k++) { 
                // Summation of L(i, j) * U(j, k) 
                T sum = 0; 
                for (j = 0; j < i; j++) 
                    sum += L.val(i, j) * U.val(j, k); 
    
                // Evaluating U(i, k) 
                *U.data(i, k) = this->val(i, k) - sum; 
            } 
    
            // Lower Triangular 
            for (k = i; k < N; k++) { 
                if (i == k) 
                    *L.data(i, i) = 1; // Diagonal as 1 
                else { 
                    // Summation of L(k, j) * U(j, i) 
                    T sum = 0; 
                    for (j = 0; j < i; j++) 
                        sum += L.val(k, j) * U.val(j, i); 
    
                    // Evaluating L(k, i) 
                    *L.data(k, i) = (this->val(k, i) - sum) / U.val(i, i); 
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

    void rotation(const _Vector<T, 3> & angles) {
        _Matrix3x3<T> R1, R2;
        this->rotation_x(angles.val(0));
        R1.rotation_y(angles.val(1));
        this->mul_mat(R1, R2);
        R1.rotation_z(angles.val(2));
        R2.mul_mat(R1, *this);
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
	_Matrix4x4(T a11, T a12, T a13, T a14, T a21, T a22, T a23, T a24, 
               T a31, T a32, T a33, T a34, T a41, T a42, T a43, T a44) 
        : _Matrix<T, 4, 4>((_DataContainerBase<T>&)*this) {
        T * p = this->data(); 
		*(p++) = a11;
		*(p++) = a12;
		*(p++) = a13;
		*(p++) = a14;
		*(p++) = a21;
		*(p++) = a22;
		*(p++) = a23;
		*(p++) = a24;
		*(p++) = a31;
		*(p++) = a32;
		*(p++) = a33;
		*(p++) = a34;
		*(p++) = a41;
		*(p++) = a42;
		*(p++) = a43;
		*(p++) = a44;
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

    void operator = (const _Matrix<T, 3, 3> & m) {
        rotation(m);
    }

    void set_rotation(const _Matrix<T, 3, 3> & m) {
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
    }
    void set_scale(const _Vector<T, 3> & v) {
        const T * sp = v.data();
        T * dp = this->data() + 4 * 3;
        *(dp++) = *(sp++);
        *(dp++) = *(sp++);
        *(dp++) = *(sp++);
    }
    void set_translation(const _Vector<T, 3> & v) {
        const T * sp = v.data();
        T * dp = this->data() + 3;
        *dp = *(sp++); dp += 4;
        *dp = *(sp++); dp += 4;
        *dp = *(sp++);
    }

    void homogenous(const _Vector<T, 3> & rot, const _Vector<T, 3> & trans, const _Vector<T, 3> & scale) {
        _Matrix3x3<T> R;
        R.rotation(rot);
        homogenous(R, trans, scale);
    }

    void homogenous(const _Matrix<T, 3, 3> & rot, const _Vector<T, 3> & trans, const _Vector<T, 3> & scale) {
        this->set_rotation(rot);
        this->set_translation(trans);
        this->set_scale(scale);
        this->row(3)[3] = 1;
    }

    void homogenous(const _Matrix<T, 3, 3> & rot, const _Vector<T, 3> & trans) {
        this->set_rotation(rot);
        this->set_translation(trans);
        T * dp = this->data() + 4 * 3;
        *(dp++) = 0;
        *(dp++) = 0;
        *(dp++) = 0;
        this->row(3)[3] = 1;
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
                dst.row(i)[j] = c * tmp.det();
            }
        }
        dst /= D;
        return true;
    }
};

template <class T, size_t N, size_t M> class _MatrixStatic : public _Matrix<T, N, M>, private _DataContainerStatic<T, N * M> {
public:
	_MatrixStatic() : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {}
	_MatrixStatic(T c) : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {
        this->_fill(c, N * M);
    }
	_MatrixStatic(const T * data) : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {
        this->_copy(data, N * M);
    }
	_MatrixStatic(const _Matrix<T, N, M> & m) : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {
        this->_copy(m.container, N * M);
    }

    _MatrixStatic<T, N, M> operator + (const _MatrixStatic<T, N, M> & m) const {
        _MatrixStatic<T, N, M> tmp(*this);
        tmp += m;
        return tmp;
    }
    _MatrixStatic<T, N, M> operator - (const _MatrixStatic<T, N, M> & m) const {
        _MatrixStatic<T, N, M> tmp(*this);
        tmp -= m;
        return tmp;
    }
    _MatrixStatic<T, N, M> operator * (const _MatrixStatic<T, N, M> & m) const {
        _MatrixStatic<T, N, M> tmp;
        this->mul_mat(m, tmp);
        return tmp;
    }
    _MatrixStatic<T, N, M> operator * (const T c) const {
        _MatrixStatic<T, N, M> tmp(*this);
        tmp *= c;
        return tmp;
    }
    _MatrixStatic<T, N, M> operator / (const T c) const {
        _MatrixStatic<T, N, M> tmp(*this);
        tmp /= c;
        return tmp;
    }
};

template <class T, size_t N, size_t M> class _MatrixDynamic : public _Matrix<T, N, M>, private _DataContainerDynamic<T, N * M> {
public:
	_MatrixDynamic() : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {}
	_MatrixDynamic(T c) : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {
        this->_fill(c, N * M);
    }
	_MatrixDynamic(const T * data) : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {
        this->_copy(data, N * M);
    }
	_MatrixDynamic(const _Matrix<T, N, M> & m) : _Matrix<T, N, M>((_DataContainerBase<T>&)*this) {
        this->_copy(m.container, N * M);
    }

    _MatrixDynamic<T, N, M> operator + (const _MatrixDynamic<T, N, M> & m) const {
        _MatrixDynamic<T, N, M> tmp(*this);
        tmp += m;
        return tmp;
    }
    _MatrixDynamic<T, N, M> operator - (const _MatrixDynamic<T, N, M> & m) const {
        _MatrixDynamic<T, N, M> tmp(*this);
        tmp -= m;
        return tmp;
    }
    _MatrixDynamic<T, N, M> operator * (const _MatrixDynamic<T, N, M> & m) const {
        _MatrixDynamic<T, N, M> tmp;
        this->mul_mat(m, tmp);
        return tmp;
    }
    _MatrixDynamic<T, N, M> operator * (const T c) const {
        _MatrixDynamic<T, N, M> tmp(*this);
        tmp *= c;
        return tmp;
    }
    _MatrixDynamic<T, N, M> operator / (const T c) const {
        _MatrixDynamic<T, N, M> tmp(*this);
        tmp /= c;
        return tmp;
    }
};


typedef _Matrix2x2<real_t> Matrix2x2;
typedef _Matrix3x3<real_t> Matrix3x3;
typedef _Matrix4x4<real_t> Matrix4x4;

}

#endif
