#pragma once

#include <cmath>

namespace math
{
    template <typename N, typename K>
    constexpr auto lbinom(float n, float k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }
    constexpr auto lbinom(double n, double k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }
    constexpr auto lbinom(long double n, long double k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }

    constexpr auto tbinom(float n, float k) noexcept
    {
        return std::exp(lbinom(n, k));
    }
    constexpr auto tbinom(double n, double k) noexcept
    {
        return std::exp(lbinom(n, k));
    }
    constexpr auto tbinom(long double n, long double k) noexcept
    {
        return std::exp(lbinom(n, k));
    }
}
