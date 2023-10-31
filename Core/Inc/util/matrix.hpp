#pragma once

#include <array>
#include <type_traits>

namespace math
{
    template <typename Scalar, std::size_t rows, std::size_t cols>
    struct Matrix {
        std::array<std::array<Scalar, cols>, rows> data;
        constexpr auto &operator[](std::size_t idx)
        {
            return data[idx];
        }
        constexpr auto const &operator[](std::size_t idx) const
        {
            return data[idx];
        }
        constexpr auto &operator()(std::size_t row, std::size_t col)
        {
            return (*this)[row][col];
        }
        constexpr auto const &operator()(std::size_t row, std::size_t col) const
        {
            return (*this)[row][col];
        }
        template <bool dummy = true>
        constexpr auto &operator()(
            std::enable_if_t<dummy && rows == 1, std::size_t> idx)
        {
            return (*this)[0][idx];
        }
        template <bool dummy = true>
        constexpr auto const &operator()(
            std::enable_if_t<dummy && rows == 1, std::size_t> idx) const
        {
            return (*this)[0][idx];
        }
        template <bool dummy = true>
        constexpr auto &operator()(
            std::enable_if_t<dummy && cols == 1 && rows != 1, std::size_t> idx)
        {
            return (*this)[idx][0];
        }
        template <bool dummy = true>
        constexpr auto const &operator()(
            std::enable_if_t<dummy && cols == 1 && rows != 1, std::size_t> idx)
            const
        {
            return (*this)[idx][0];
        }
        constexpr auto &operator+=(Matrix const &right)
        {
            for (std::size_t row{}; row < rows; ++row) {
                for (std::size_t col{}; col < cols; ++col) {
                    (*this)(row, col) += right(row, col);
                }
            }
            return *this;
        }
        constexpr auto &operator-=(Matrix const &right)
        {
            for (std::size_t row{}; row < rows; ++row) {
                for (std::size_t col{}; col < cols; ++col) {
                    (*this)(row, col) -= right(row, col);
                }
            }
            return *this;
        }
        constexpr auto &operator*=(Scalar const &right)
        {
            for (std::size_t row{}; row < rows; ++row) {
                for (std::size_t col{}; col < cols; ++col) {
                    (*this)(row, col) *= right;
                }
            }
            return *this;
        }
        constexpr friend auto operator+(Matrix right)
        {
            return right;
        }
        constexpr friend auto operator+(Matrix left, Matrix const &right)
        {
            left += right;
            return left;
        }
        constexpr friend auto operator-(Matrix right)
        {
            right *= -1;
            return right;
        }
        constexpr friend auto operator-(Matrix left, Matrix const &right)
        {
            left -= right;
            return left;
        }
        constexpr friend auto operator*(Matrix left, Scalar const &right)
        {
            left *= right;
            return left;
        }
        constexpr friend auto operator*(Scalar const &left, Matrix right)
        {
            right *= left;
            return right;
        }
        template <typename Scalar_,
                  std::size_t rows_,
                  std::size_t size_,
                  std::size_t cols_>
        constexpr friend auto operator*(Matrix<Scalar_, rows_, size_> const &left,
                                        Matrix<Scalar_, size_, cols_> const &right);
    };
    template <typename Scalar, std::size_t rows, std::size_t size, std::size_t cols>
    constexpr auto operator*(Matrix<Scalar, rows, size> const &left,
                             Matrix<Scalar, size, cols> const &right)
    {
        Matrix<Scalar, rows, cols> ret{};
        for (std::size_t row{}; row < rows; ++row) {
            for (std::size_t col{}; col < cols; ++col) {
                for (std::size_t idx{}; idx < size; ++idx) {
                    ret(row, col) = left(row, idx) * right(idx, col);
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
} // namespace math
