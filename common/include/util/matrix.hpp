#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

#include "math.hpp"

namespace math
{
    /**
     * @brief Mathematical matrix
     *
     * @tparam Scalar type of scalar
     * @tparam rows number of rows
     * @tparam cols number of columns
     */
    template <typename Scalar, std::size_t rows, std::size_t cols>
    struct Matrix
    {
        template <typename Scalar_,
                  std::size_t rows_,
                  std::size_t size_,
                  std::size_t cols_>
        constexpr friend auto operator* [[nodiscard]] (Matrix<Scalar_, rows_, size_> const &left,
                                                       Matrix<Scalar_, size_, cols_> const &right) noexcept;

        /**
         * @brief Matrix data
         */
        std::array<std::array<Scalar, cols>, rows> data;

        /**
         * @brief Get a row by index
         *
         * @param row row index
         * @return a reference to the row
         */
        constexpr auto &operator[] [[nodiscard]] (std::size_t row) noexcept
        {
            return data[row];
        }

        /**
         * @brief Get a row by index
         *
         * @param row row index
         * @return a const reference to the row
         */
        constexpr auto const &operator[] [[nodiscard]] (std::size_t row) const noexcept
        {
            return data[row];
        }

        /**
         * @brief Get an element by row and column indices
         *
         * @param row row index
         * @param col column index
         * @return a reference to the element
         */
        constexpr auto &operator() [[nodiscard]] (std::size_t row, std::size_t col) noexcept
        {
            return (*this)[row][col];
        }

        /**
         * @brief Get an element by row and column indices
         *
         * @param row row index
         * @param col column index
         * @return a const reference to the element
         */
        constexpr auto const &operator() [[nodiscard]] (std::size_t row, std::size_t col) const noexcept
        {
            return (*this)[row][col];
        }

        /**
         * @brief Get an element from a row vector by index
         *
         * @tparam dummy dummy to defer evaluation of `std::enable_if_t`
         * @param col column index
         * @return a reference to the element
         */
        template <bool dummy = true>
        constexpr auto &operator() [[nodiscard]] (
            std::enable_if_t<dummy && rows == 1, std::size_t> col) noexcept
        {
            return (*this)[0][col];
        }

        /**
         * @brief Get an element from a row vector by index
         *
         * @tparam dummy dummy to defer evaluation of `std::enable_if_t`
         * @param col column index
         * @return a const reference to the element
         */
        template <bool dummy = true>
        constexpr auto const &operator() [[nodiscard]] (
            std::enable_if_t<dummy && rows == 1, std::size_t> col) const noexcept
        {
            return (*this)[0][col];
        }

        /**
         * @brief Get an element from a column vector by index
         *
         * @tparam dummy dummy to defer evaluation of `std::enable_if_t`
         * @param row row index
         * @return a reference to the element
         */
        template <bool dummy = true>
        constexpr auto &operator() [[nodiscard]] (
            std::enable_if_t<dummy && cols == 1 && rows != 1, std::size_t> row) noexcept
        {
            return (*this)[row][0];
        }

        /**
         * @brief Get an element from a column vector by index
         *
         * @tparam dummy dummy to defer evaluation of `std::enable_if_t`
         * @param row row index
         * @return a const reference to the element
         */
        template <bool dummy = true>
        constexpr auto const &operator() [[nodiscard]] (
            std::enable_if_t<dummy && cols == 1 && rows != 1, std::size_t> row) const noexcept
        {
            return (*this)[row][0];
        }

        /**
         * @brief Add a matrix to this matrix
         *
         * @param right matrix in the right operand
         * @return `*this`
         */
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

        /**
         * @brief Subtract a matrix to this matrix
         *
         * @param right matrix in the right operand
         * @return `*this`
         */
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

        /**
         * @brief Multiply this matrix by a scalar
         *
         * @param right scalar in the right operand
         * @return `*this`
         */
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

        /**
         * @brief Divide this matrix by a scalar
         *
         * @param right scalar in the right operand
         * @return `*this`
         */
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

        /**
         * @brief Return a matrix unchanged
         *
         * @param right matrix in the right operand
         * @return a unchanged matrix
         */
        constexpr friend auto operator+ [[nodiscard]] (Matrix right) noexcept
        {
            return right;
        }

        /**
         * @brief Add two matrices
         *
         * @param left matrix in the left operand
         * @param right matrix in the right operand
         * @return a matrix sum
         */
        constexpr friend auto operator+ [[nodiscard]] (Matrix left, Matrix const &right) noexcept
        {
            left += right;
            return left;
        }

        /**
         * @brief Negate a matrix
         *
         * @param right matrix in the right operand
         * @return a negated matrix
         */
        constexpr friend auto operator- [[nodiscard]] (Matrix right) noexcept
        {
            right *= static_cast<Scalar>(-1);
            return right;
        }

        /**
         * @brief Subtract the right matrix from the left matrix
         *
         * @param left matrix in the left operand
         * @param right matrix in the right operand
         * @return a subtracted matrix
         */
        constexpr friend auto operator- [[nodiscard]] (Matrix left, Matrix const &right) noexcept
        {
            left -= right;
            return left;
        }

        /**
         * @brief Multiply a matrix by a scalar
         *
         * @param left matrix in the left operand
         * @param right scalar in the right operand
         * @return a multiplied matrix
         */
        constexpr friend auto operator* [[nodiscard]] (Matrix left, Scalar const &right) noexcept
        {
            left *= right;
            return left;
        }

        /**
         * @brief Multiply a matrix by a scalar
         *
         * @param left scalar in the left operand
         * @param right matrix in the right operand
         * @return a multiplied matrix
         */
        constexpr friend auto operator* [[nodiscard]] (Scalar const &left, Matrix right) noexcept
        {
            right *= left;
            return right;
        }

        /**
         * @brief Divide a matrix by a scalar
         *
         * @param left matrix in the left operand
         * @param right scalar in the right operand
         * @return a divided matrix
         */
        constexpr friend auto operator/ [[nodiscard]] (Matrix left, Scalar const &right) noexcept
        {
            left /= right;
            return left;
        }

        /**
         * @brief Multiply this square matrix by a square matrix of the same size
         *
         * @tparam dummy dummy to defer evaluation of `std::enable_if_t`
         * @param right matrix in the right operand
         * @return `*this`
         */
        template <bool dummy = true>
        constexpr auto &operator*=(std::enable_if_t<dummy && rows == cols, Matrix> const &right) noexcept
        {
            *this = *this * right;
            return *this;
        }

        /**
         * @brief Return the transpose of this matrix
         *
         * @return a transposed matrix
         */
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

    /**
     * @brief Multiply two matrices
     *
     * @tparam Scalar type of scalar
     * @tparam rows number of rows in the resulting matrix
     * @tparam size common size between the two matrices
     * @tparam cols number of columns in the resulting matrix
     * @param left matrix in the left operand
     * @param right matrix in the right operand
     * @return a multiplied matrix
     */
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

    /**
     * @brief Mathematical row vector
     *
     * @tparam Scalar type of scalar
     * @tparam size number of elements
     */
    template <typename Scalar, std::size_t size>
    using RowVector = Matrix<Scalar, 1, size>;

    /**
     * @brief Mathematical square matrix
     *
     * @tparam Scalar type of scalar
     * @tparam size number of rows and columns
     */
    template <typename Scalar, std::size_t size>
    using SquareMatrix = Matrix<Scalar, size, size>;

    /**
     * @brief Mathematical column vector
     *
     * @tparam Scalar type of scalar
     * @tparam size number of elements
     */
    template <typename Scalar, std::size_t size>
    using Vector = Matrix<Scalar, size, 1>;

    /**
     * @brief Identity matrix
     *
     * @tparam Scalar type of scalar
     * @tparam size number of rows and columns
     */
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

    /**
     * @brief Calculate the dot product of two column vectors
     *
     * @tparam Scalar type of scalar
     * @tparam size number of elements
     * @param left vector in the left operand
     * @param right vector in the right operand
     * @return the dot product
     */
    template <typename Scalar, std::size_t size>
    constexpr auto dot_product [[nodiscard]] (Vector<Scalar, size> const &left, Vector<Scalar, size> const &right) noexcept
    {
        return (left.transpose() * right)(0);
    }

    /**
     * @brief Calculate the dot product of two row vectors
     *
     * @tparam Scalar type of scalar
     * @tparam size number of elements
     * @param left vector in the left operand
     * @param right vector in the right operand
     * @return the dot product
     */
    template <typename Scalar, std::size_t size>
    constexpr auto dot_product [[nodiscard]] (RowVector<Scalar, size> const &left, RowVector<Scalar, size> const &right) noexcept
    {
        return (left * right.transpose())(0);
    }

    /**
     * @brief Calculate the magnitude of a column vector
     *
     * @tparam Scalar type of scalar
     * @tparam size number of elements
     * @param vector vector
     * @return the magnitude
     */
    template <typename Scalar, std::size_t size>
    constexpr auto magnitude [[nodiscard]] (Vector<Scalar, size> const &vector) noexcept
    {
        return std::sqrt(dot_product(vector, vector));
    }

    /**
     * @brief Calculate the magnitude of a row vector
     *
     * @tparam Scalar type of scalar
     * @tparam size number of elements
     * @param vector vector
     * @return the magnitude
     */
    template <typename Scalar, std::size_t size>
    constexpr auto magnitude [[nodiscard]] (RowVector<Scalar, size> const &vector) noexcept
    {
        return std::sqrt(dot_product(vector, vector));
    }

    /**
     * @brief Calculate the unit vector for a column vector
     *
     * @tparam Scalar type of scalar
     * @tparam size number of elements
     * @param vector vector
     * @return a unit vector in the direction of `vector`
     */
    template <typename Scalar, std::size_t size>
    constexpr auto unit_vector [[nodiscard]] (Vector<Scalar, size> vector) noexcept
    {
        auto const mag{magnitude(vector)};
        if (std::abs(mag - 0.) < math::epsilon)
        {
            return decltype(vector){};
        }
        vector /= magnitude(vector);
        return vector;
    }

    /**
     * @brief Calculate the unit vector for a row vector
     *
     * @tparam Scalar type of scalar
     * @tparam size number of elements
     * @param vector vector
     * @return a unit vector in the direction of `vector`
     */
    template <typename Scalar, std::size_t size>
    constexpr auto unit_vector [[nodiscard]] (RowVector<Scalar, size> vector) noexcept
    {
        auto const mag{magnitude(vector)};
        if (std::abs(mag - 0.) < math::epsilon)
        {
            return decltype(vector){};
        }
        vector /= magnitude(vector);
        return vector;
    }

    /**
     * @brief Get the 2D rotation matrix for the specified angle
     *
     * @tparam Scalar type of scalar
     * @param angle rotation angle
     * @return a 2D rotation matrix
     */
    template <typename Scalar>
    constexpr auto rotation_matrix2 [[nodiscard]] (Scalar angle) noexcept -> SquareMatrix<Scalar, 2>
    {
        return {std::cos(angle), -std::sin(angle),
                std::sin(angle), std::cos(angle)};
    }

    /**
     * @brief Get the angle of the 2D rotation matrix
     *
     * @tparam Scalar type of scalar
     * @param matrix 2D rotation matrix
     * @return angle
     */
    template <typename Scalar>
    constexpr auto rotation_matrix2_angle [[nodiscard]] (SquareMatrix<Scalar, 2> const &matrix) noexcept
    {
        const auto point{matrix * Vector<Scalar, 2>{static_cast<Scalar>(1), static_cast<Scalar>(0)}};
        return std::atan2(point(1), point(0));
    }

    /**
     * @brief Orthgonalize a 2D rotation matrix to avoiding accumulating rounding errors after matrix operations
     *
     * @tparam Scalar type of scalar
     * @param matrix 2D matrix to orthgonalize
     * @return an orthgonalized 2D rotation matrix
     */
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
