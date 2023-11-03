#pragma once

#include <cmath>

namespace math
{
    constexpr auto const epsilon{1e-6};

    constexpr auto lbinom [[nodiscard]] (float n, float k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }
    constexpr auto lbinom [[nodiscard]] (double n, double k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }
    constexpr auto lbinom [[nodiscard]] (long double n, long double k) noexcept
    {
        return std::lgamma(n + 1) - std::lgamma(n - k + 1) - std::lgamma(k + 1);
    }

    constexpr auto tbinom [[nodiscard]] (float n, float k) noexcept
    {
        return std::exp(lbinom(n, k));
    }
    constexpr auto tbinom [[nodiscard]] (double n, double k) noexcept
    {
        return std::exp(lbinom(n, k));
    }
    constexpr auto tbinom [[nodiscard]] (long double n, long double k) noexcept
    {
        return std::exp(lbinom(n, k));
    }
}
