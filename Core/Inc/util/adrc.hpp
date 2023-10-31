#pragma once

#include <cmath>
#include <cstddef>
#include <utility>

#include "matrix.hpp"

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
        constexpr static auto const B{[]() {
            math::Vector<Scalar, order> ret{};
            ret(order - 2) = 1;
            return ret;
        }()};
        constexpr static math::RowVector<Scalar, order> const C{1};
        constexpr static auto L(Scalar convergence)
        {
            math::Vector<Scalar, order> ret{};
            for (std::size_t ii{}; ii < order; ++ii) {
                ret(ii) = 1;
                for (std::size_t jj{order}; jj > ii + 1; --jj) {
                    ret(ii) *= jj;
                }
                ret(ii) *= std::pow(convergence, static_cast<Scalar>(ii + 1));
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
                  auto const &Lc{L(convergence)};
                  decltype(m_B) ret{};
                  for (std::size_t ii{}; ii < order; ++ii) {
                      ret[ii] = {B(ii), Lc(ii)};
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
                  for (std::size_t ii{}; ii < order - 1; ++ii) {
                      ret(ii) = 1;
                      for (std::size_t jj{order - 1}; jj > order - ii - 1; --jj) {
                          ret(ii) *= jj;
                      }
                      ret(ii) *= std::pow(convergence,
                                          static_cast<Scalar>(order - ii - 1));
                  }
                  ret(order - 1) = 1;
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
            return control(target);
        }
    };
    using ADRC2f = ADRC<float, 2>;
    using ADRC3f = ADRC<float, 3>;
} // namespace control
