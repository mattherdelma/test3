// matrix.hpp - Minimal fixed-size linear algebra for the PTZ control stack.
//
// Header-only, dependency-free. Dimensions are compile-time template
// parameters so the Kalman filter math is fully bounds-checked at compile
// time and needs no heap allocation. Only the operations required by the
// control stack are implemented (multiply, transpose, add/sub, small inverse
// via Gauss-Jordan).
#ifndef PTZ_MATRIX_HPP
#define PTZ_MATRIX_HPP

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

namespace ptz {

template <std::size_t Rows, std::size_t Cols>
class Matrix {
public:
    std::array<double, Rows * Cols> data{};

    Matrix() = default;

    Matrix(std::initializer_list<double> values) {
        std::size_t i = 0;
        for (double v : values) {
            if (i >= Rows * Cols) break;
            data[i++] = v;
        }
    }

    double& operator()(std::size_t r, std::size_t c) { return data[r * Cols + c]; }
    double operator()(std::size_t r, std::size_t c) const { return data[r * Cols + c]; }

    static Matrix Zero() { return Matrix{}; }

    static Matrix Identity() {
        static_assert(Rows == Cols, "Identity requires a square matrix");
        Matrix m;
        for (std::size_t i = 0; i < Rows; ++i) m(i, i) = 1.0;
        return m;
    }

    Matrix operator+(const Matrix& o) const {
        Matrix r;
        for (std::size_t i = 0; i < Rows * Cols; ++i) r.data[i] = data[i] + o.data[i];
        return r;
    }

    Matrix operator-(const Matrix& o) const {
        Matrix r;
        for (std::size_t i = 0; i < Rows * Cols; ++i) r.data[i] = data[i] - o.data[i];
        return r;
    }

    Matrix operator*(double s) const {
        Matrix r;
        for (std::size_t i = 0; i < Rows * Cols; ++i) r.data[i] = data[i] * s;
        return r;
    }

    template <std::size_t OtherCols>
    Matrix<Rows, OtherCols> operator*(const Matrix<Cols, OtherCols>& o) const {
        Matrix<Rows, OtherCols> r;
        for (std::size_t i = 0; i < Rows; ++i) {
            for (std::size_t k = 0; k < Cols; ++k) {
                const double a = (*this)(i, k);
                if (a == 0.0) continue;
                for (std::size_t j = 0; j < OtherCols; ++j) {
                    r(i, j) += a * o(k, j);
                }
            }
        }
        return r;
    }

    Matrix<Cols, Rows> transpose() const {
        Matrix<Cols, Rows> r;
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j) r(j, i) = (*this)(i, j);
        return r;
    }

    // Gauss-Jordan inverse for small square matrices (n <= a few).
    Matrix inverse() const {
        static_assert(Rows == Cols, "inverse requires a square matrix");
        Matrix a = *this;
        Matrix inv = Identity();
        for (std::size_t col = 0; col < Rows; ++col) {
            // Partial pivot for numerical stability.
            std::size_t pivot = col;
            double best = std::fabs(a(col, col));
            for (std::size_t r = col + 1; r < Rows; ++r) {
                double v = std::fabs(a(r, col));
                if (v > best) { best = v; pivot = r; }
            }
            if (best < 1e-12) {
                throw std::runtime_error("Matrix::inverse: singular matrix");
            }
            if (pivot != col) {
                for (std::size_t j = 0; j < Rows; ++j) {
                    std::swap(a(col, j), a(pivot, j));
                    std::swap(inv(col, j), inv(pivot, j));
                }
            }
            const double diag = a(col, col);
            for (std::size_t j = 0; j < Rows; ++j) {
                a(col, j) /= diag;
                inv(col, j) /= diag;
            }
            for (std::size_t r = 0; r < Rows; ++r) {
                if (r == col) continue;
                const double factor = a(r, col);
                if (factor == 0.0) continue;
                for (std::size_t j = 0; j < Rows; ++j) {
                    a(r, j) -= factor * a(col, j);
                    inv(r, j) -= factor * inv(col, j);
                }
            }
        }
        return inv;
    }
};

}  // namespace ptz

#endif  // PTZ_MATRIX_HPP
