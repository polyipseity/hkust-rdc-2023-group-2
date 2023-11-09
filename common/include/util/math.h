#pragma once

#include <math.h>

#include "globals.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief A small number
     */
    CONSTEXPR double const math_epsilon = 1e-6;

    /**
     * @brief Pi
     */
    CONSTEXPR double const math_pi = 3.14159265358979323846;

    /**
     * @brief Tau
     */
    CONSTEXPR double const math_tau = 2. * 3.14159265358979323846;

    /**
     * @brief Calculate the natural logarithm of the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the natural logarithm of the specified binominal coefficient
     */
    __attribute__((warn_unused_result)) inline float math_lbinomf(float n, float k) NOEXCEPT
    {
        return lgammaf(n + 1) - lgammaf(n - k + 1) - lgammaf(k + 1);
    }

    /**
     * @brief Calculate the natural logarithm of the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the natural logarithm of the specified binominal coefficient
     */
    __attribute__((warn_unused_result)) inline double math_lbinom(double n, double k) NOEXCEPT
    {
        return lgamma(n + 1) - lgamma(n - k + 1) - lgamma(k + 1);
    }

    /**
     * @brief Calculate the natural logarithm of the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the natural logarithm of the specified binominal coefficient
     */
    __attribute__((warn_unused_result)) inline long double math_lbinoml(long double n, long double k) NOEXCEPT
    {
        return lgammal(n + 1) - lgammal(n - k + 1) - lgammal(k + 1);
    }

    /**
     * @brief Calculate the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the specified binominal coefficient
     */
    __attribute__((warn_unused_result)) inline float math_tbinomf(float n, float k) NOEXCEPT
    {
        return expf(math_lbinomf(n, k));
    }

    /**
     * @brief Calculate the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the specified binominal coefficient
     */
    __attribute__((warn_unused_result)) inline double math_tbinom(double n, double k) NOEXCEPT
    {
        return exp(math_lbinom(n, k));
    }

    /**
     * @brief Calculate the specified binominal coefficient
     *
     * @param n n
     * @param k k
     * @return the specified binominal coefficient
     */
    __attribute__((warn_unused_result)) inline long double math_tbinoml(long double n, long double k) NOEXCEPT
    {
        return expl(math_lbinoml(n, k));
    }

#ifdef __cplusplus
}
#endif
