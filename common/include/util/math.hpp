#pragma once

#include <cmath>

namespace math
{
    /**
     * @brief A small number
     */
    constexpr auto const epsilon{1e-6};

    /**
     * @brief Calculate the natural logarithm of the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the natural logarithm of the specified binominal coefficient
     */
    constexpr auto lbinom [[nodiscard]] (float n, float k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }

    /**
     * @brief Calculate the natural logarithm of the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the natural logarithm of the specified binominal coefficient
     */
    constexpr auto lbinom [[nodiscard]] (double n, double k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }

    /**
     * @brief Calculate the natural logarithm of the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the natural logarithm of the specified binominal coefficient
     */
    constexpr auto lbinom [[nodiscard]] (long double n, long double k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }

    /**
     * @brief Calculate the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the specified binominal coefficient
     */
    constexpr auto tbinom [[nodiscard]] (float n, float k) noexcept
    {
        return std::exp(lbinom(n, k));
    }

    /**
     * @brief Calculate the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the specified binominal coefficient
     */
    constexpr auto tbinom [[nodiscard]] (double n, double k) noexcept
    {
        return std::exp(lbinom(n, k));
    }

    /**
     * @brief Calculate the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the specified binominal coefficient
     */
    constexpr auto tbinom [[nodiscard]] (long double n, long double k) noexcept
    {
        return std::exp(lbinom(n, k));
    }
}
