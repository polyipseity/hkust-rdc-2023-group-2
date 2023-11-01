#pragma once

#include <cmath>
#include <cstddef>
#include <utility>

#include "matrix.hpp"
#include "util/math.hpp"

namespace control
{
    template <typename Scalar, std::size_t order>
    class ESO
    {
    public:
        using State = math::Vector<Scalar, order>;
        constexpr static auto const A{[]() {
            math::SquareMatrix<Scalar, order> ret{};
            for (std::size_t ii{1}; ii < order; ++ii) {
                ret(ii - 1, ii) = 1;
            }
            return ret;
        }()};
        constexpr static auto B(Scalar gain)
        {
            math::Vector<Scalar, order> ret{};
            ret(order - 2) = gain;
            return ret;
        }
        constexpr static math::RowVector<Scalar, order> const C{1};
        constexpr static auto L(Scalar convergence)
        {
            math::Vector<Scalar, order> ret{};
            for (std::size_t ii{}; ii < order; ++ii) {
                // (s+w)^n
                // n=3, (s+w)^3=s^3+3s^2*w+3sw^2+w^3
                ret(ii) = math::tbinom(static_cast<Scalar>(order), static_cast<Scalar>(ii + 1)) * std::pow(convergence, static_cast<Scalar>(ii + 1));
            }
            return ret;
        }
        State m_state;

    private:
        math::SquareMatrix<Scalar, order> m_A;
        math::Matrix<Scalar, order, 2> m_B;

    public:
        constexpr ESO(Scalar gain, Scalar convergence, State state)
            : m_state{std::move(state)},
              m_A{A - L(convergence) * C},
              m_B{[convergence, gain]() {
                  auto const &B0(B(gain)), &L0{L(convergence)};
                  decltype(m_B) ret{};
                  for (std::size_t ii{}; ii < order; ++ii) {
                      ret[ii] = {B0(ii), L0(ii)};
                  }
                  return ret;
              }()}
        {
        }
        constexpr auto update(Scalar input, Scalar output, Scalar dt)
        {
            m_state += dt * (m_A * m_state + m_B * math::Vector<Scalar, 2>{
                                                       input, output - m_state(0)});
        }
    };
    using ESO2f = ESO<float, 2>;
    using ESO3f = ESO<float, 3>;
    using ESO2d = ESO<double, 2>;
    using ESO3d = ESO<double, 3>;

    template <typename Scalar, std::size_t order>
    class ADRC
    {
    public:
        using Observer = ESO<Scalar, order>;
        using State    = typename Observer::State;
        constexpr static Scalar const observer_convergence_factor{10};
        Observer m_observer;

    private:
        Scalar m_gain;
        math::RowVector<Scalar, order> m_K;

    public:
        constexpr ADRC(Scalar gain, Scalar convergence, State state)
            : m_observer{gain,
                         convergence * observer_convergence_factor,
                         std::move(state)},
              m_gain{gain},
              m_K{[convergence]() {
                  decltype(m_K) ret{};
                  for (std::size_t ii{}; ii < order; ++ii) {
                      // (s+w)^(n-1)
                      // n=3, (s+w)^2=w^2+2ws+s^2
                      ret(ii) = math::tbinom(static_cast<Scalar>(order - 1), static_cast<Scalar>(order - 1 - ii)) * std::pow(convergence, static_cast<Scalar>(order - 1 - ii));
                  }
                  return ret;
              }()}
        {
        }
        constexpr auto control(Scalar target) const
        {
            auto const &state{m_observer.m_state};
            return (m_K * (State{target} - state))(0) / m_gain;
        }
        constexpr auto update(Scalar target,
                              Scalar input,
                              Scalar output,
                              Scalar dt) // Synchronize with `PID::update`
        {
            m_observer.update(input, output, dt);
            return control(target / static_cast<Scalar>(2));
        }
    };
    using ADRC2f = ADRC<float, 2>;
    using ADRC3f = ADRC<float, 3>;
    using ADRC2d = ADRC<double, 2>;
    using ADRC3d = ADRC<double, 3>;
} // namespace control
