#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

#include "math.hpp"

namespace math
{
    template <typename Scalar, std::size_t rows, std::size_t cols>
    struct Matrix
    {
        std::array<std::array<Scalar, cols>, rows> data;
        constexpr auto &operator[] [[nodiscard]] (std::size_t idx) noexcept
        {
            return data[idx];
        }
        constexpr auto const &operator[] [[nodiscard]] (std::size_t idx) const noexcept
        {
            return data[idx];
        }
        constexpr auto &operator() [[nodiscard]] (std::size_t row, std::size_t col) noexcept
        {
            return (*this)[row][col];
        }
        constexpr auto const &operator() [[nodiscard]] (std::size_t row, std::size_t col) const noexcept
        {
            return (*this)[row][col];
        }
        template <bool dummy = true>
        constexpr auto &operator() [[nodiscard]] (
            std::enable_if_t<dummy && rows == 1, std::size_t> idx) noexcept
        {
            return (*this)[0][idx];
        }
        template <bool dummy = true>
        constexpr auto const &operator() [[nodiscard]] (
            std::enable_if_t<dummy && rows == 1, std::size_t> idx) const noexcept
        {
            return (*this)[0][idx];
        }
        template <bool dummy = true>
        constexpr auto &operator() [[nodiscard]] (
            std::enable_if_t<dummy && cols == 1 && rows != 1, std::size_t> idx) noexcept
        {
            return (*this)[idx][0];
        }
        template <bool dummy = true>
        constexpr auto const &operator() [[nodiscard]] (
            std::enable_if_t<dummy && cols == 1 && rows != 1, std::size_t> idx) const noexcept
        {
            return (*this)[idx][0];
        }
        constexpr auto &operator+=(Matrix const &right) noexcept
        {
            for (std::size_t row{}; row < rows; ++row)
            {
                for (std::size_t col{}; col < cols; ++col)
                {
                    (*this)(row, col) += right(row, col);
                }
            }
            return *this;
        }
        constexpr auto &operator-=(Matrix const &right) noexcept
        {
            for (std::size_t row{}; row < rows; ++row)
            {
                for (std::size_t col{}; col < cols; ++col)
                {
                    (*this)(row, col) -= right(row, col);
                }
            }
            return *this;
        }
        constexpr auto &operator*=(Scalar const &right) noexcept
        {
            for (std::size_t row{}; row < rows; ++row)
            {
                for (std::size_t col{}; col < cols; ++col)
                {
                    (*this)(row, col) *= right;
                }
            }
            return *this;
        }
        constexpr auto &operator/=(Scalar const &right) noexcept
        {
            for (std::size_t row{}; row < rows; ++row)
            {
                for (std::size_t col{}; col < cols; ++col)
                {
                    (*this)(row, col) /= right;
                }
            }
            return *this;
        }
        constexpr friend auto operator+ [[nodiscard]] (Matrix right) noexcept
        {
            return right;
        }
        constexpr friend auto operator+ [[nodiscard]] (Matrix left, Matrix const &right) noexcept
        {
            left += right;
            return left;
        }
        constexpr friend auto operator- [[nodiscard]] (Matrix right) noexcept
        {
            right *= static_cast<Scalar>(-1);
            return right;
        }
        constexpr friend auto operator- [[nodiscard]] (Matrix left, Matrix const &right) noexcept
        {
            left -= right;
            return left;
        }
        constexpr friend auto operator* [[nodiscard]] (Matrix left, Scalar const &right) noexcept
        {
            left *= right;
            return left;
        }
        constexpr friend auto operator* [[nodiscard]] (Scalar const &left, Matrix right) noexcept
        {
            right *= left;
            return right;
        }
        constexpr friend auto operator/ [[nodiscard]] (Matrix left, Scalar const &right) noexcept
        {
            left /= right;
            return left;
        }
        template <typename Scalar_,
                  std::size_t rows_,
                  std::size_t size_,
                  std::size_t cols_>
        constexpr friend auto operator* [[nodiscard]] (Matrix<Scalar_, rows_, size_> const &left,
                                                       Matrix<Scalar_, size_, cols_> const &right) noexcept;
        template <bool dummy = true>
        constexpr auto &operator*=(std::enable_if_t<dummy && rows == cols, Matrix> const &right) noexcept
        {
            *this = *this * right;
            return *this;
        }
        constexpr auto transpose [[nodiscard]] () const noexcept
        {
            Matrix<Scalar, cols, rows> ret{};
            for (std::size_t row{}; row < rows; ++row)
            {
                for (std::size_t col{}; col < cols; ++col)
                {
                    ret(col, row) = (*this)(row, col);
                }
            }
            return ret;
        }
    };
    template <typename Scalar, std::size_t rows, std::size_t size, std::size_t cols>
    constexpr auto operator* [[nodiscard]] (Matrix<Scalar, rows, size> const &left,
                                            Matrix<Scalar, size, cols> const &right) noexcept
    {
        Matrix<Scalar, rows, cols> ret{};
        for (std::size_t row{}; row < rows; ++row)
        {
            for (std::size_t col{}; col < cols; ++col)
            {
                for (std::size_t idx{}; idx < size; ++idx)
                {
                    ret(row, col) += left(row, idx) * right(idx, col);
                }
            }
        }
        return ret;
    }

    template <typename Scalar, std::size_t size>
    using RowVector = Matrix<Scalar, 1, size>;
    template <typename Scalar, std::size_t size>
    using SquareMatrix = Matrix<Scalar, size, size>;
    template <typename Scalar, std::size_t size>
    using Vector = Matrix<Scalar, size, 1>;

    template <typename Scalar, std::size_t size>
    constexpr auto const identity{[]()
                                  {
                                      SquareMatrix<Scalar, size> ret{};
                                      for (std::size_t ii{}; ii < size; ++ii)
                                      {
                                          ret(ii, ii) = static_cast<Scalar>(1);
                                      }
                                      return ret;
                                  }()};
    template <typename Scalar, std::size_t size>
    constexpr auto dot_product [[nodiscard]] (Vector<Scalar, size> const &left, Vector<Scalar, size> const &right) noexcept
    {
        return (left.transpose() * right)(0);
    }
    template <typename Scalar, std::size_t size>
    constexpr auto dot_product [[nodiscard]] (RowVector<Scalar, size> const &left, RowVector<Scalar, size> const &right) noexcept
    {
        return (left * right.transpose())(0);
    }
    template <typename Scalar, std::size_t size>
    constexpr auto magnitude [[nodiscard]] (Vector<Scalar, size> const &vector) noexcept
    {
        return std::sqrt(dot_product(vector, vector));
    }
    template <typename Scalar, std::size_t size>
    constexpr auto magnitude [[nodiscard]] (RowVector<Scalar, size> const &vector) noexcept
    {
        return std::sqrt(dot_product(vector, vector));
    }
    template <typename Scalar, std::size_t size>
    constexpr auto unit_vector [[nodiscard]] (Vector<Scalar, size> vector) noexcept
    {
        auto const mag{magnitude(vector)};
        if (std::abs(mag - 0.) >= math::epsilon)
        {
            vector /= magnitude(vector);
        }
        return vector;
    }
    template <typename Scalar, std::size_t size>
    constexpr auto unit_vector [[nodiscard]] (RowVector<Scalar, size> vector) noexcept
    {
        auto const mag{magnitude(vector)};
        if (std::abs(mag - 0.) >= math::epsilon)
        {
            vector /= magnitude(vector);
        }
        return vector;
    }
    template <typename Scalar>
    constexpr auto rotation_matrix2 [[nodiscard]] (Scalar rotation) noexcept -> SquareMatrix<Scalar, 2>
    {
        return {std::cos(rotation), -std::sin(rotation),
                std::sin(rotation), std::cos(rotation)};
    }
    template <typename Scalar>
    constexpr auto orthogonalize_rotation_matrix2 [[nodiscard]] (SquareMatrix<Scalar, 2> matrix) noexcept
    {
        // https://stackoverflow.com/a/23082112

        RowVector<Scalar, 2> const x{matrix[0]}, y{matrix[1]};
        auto const error{dot_product(x, y)};
        auto const x_ort{x - y * (error / 2.)}, y_ort{y - x * (error / 2.)};
        matrix[0] = (x_ort * .5 * (3 - dot_product(x_ort, x_ort)))[0];
        matrix[1] = (y_ort * .5 * (3 - dot_product(y_ort, y_ort)))[0];
        return matrix;
    }
} // namespace math
