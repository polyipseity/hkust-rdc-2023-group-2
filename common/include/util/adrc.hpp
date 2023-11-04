#pragma once

#include <cmath>
#include <cstddef>
#include <utility>

#include "matrix.hpp"
#include "util/math.hpp"

namespace control
{
    /**
     * @brief A state observer tracking the system state and unknown disturbances
     *
     * @tparam Scalar type of scalar
     * @tparam order system order
     */
    template <typename Scalar, std::size_t order>
    class ESO
    {
    public:
        /**
         * @brief State type
         */
        using State = math::Vector<Scalar, order>;

        /**
         * @brief A matrix relating the current state to the next state
         */
        constexpr static auto const A{[]()
                                      {
                                          math::SquareMatrix<Scalar, order> ret{};
                                          for (std::size_t ii{1}; ii < order; ++ii)
                                          {
                                              ret(ii - 1, ii) = 1;
                                          }
                                          return ret;
                                      }()};

        /**
         * @brief A matrix relating the input to the next state
         *
         * @param gain rate of change in the (order - 1)-th state (velocity for 2nd order, acceleration for 3rd order) per a unit of input, i.e. the rate of change in the (order - 1)-th state divided by the input
         * @return the matrix `B` for the specified gain
         */
        constexpr static auto B [[nodiscard]] (Scalar gain) noexcept
        {
            math::Vector<Scalar, order> ret{};
            ret(order - 2) = gain;
            return ret;
        }

        /**
         * @brief A matrix relating the state to the output
         */
        constexpr static math::RowVector<Scalar, order> const C{1};

        /**
         * @brief A matrix determining the convergence of the estimated state to the state
         *
         * @param convergence convergence rate of the estimated state to the actual state
         * @return the observer convergence matrix
         */
        constexpr static auto L [[nodiscard]] (Scalar convergence) noexcept
        {
            math::Vector<Scalar, order> ret{};
            for (std::size_t ii{}; ii < order; ++ii)
            {
                // (s+w)^n
                // n=3, (s+w)^3=s^3+3s^2*w+3sw^2+w^3
                ret(ii) = math::tbinom(static_cast<Scalar>(order), static_cast<Scalar>(ii + 1)) * std::pow(convergence, static_cast<Scalar>(ii + 1));
            }
            return ret;
        }

        /**
         * @brief The estimated state
         */
        State m_state;

    private:
        /**
         * @brief A matrix calculated from `A - L * C`. It can be treated as the `A` for the estimated state
         */
        math::SquareMatrix<Scalar, order> m_A;

        /**
         * @brief A matrix consisting of `B` in the first column and `L` in the second column. It can be treated as the `B` for the estimated state
         */
        math::Matrix<Scalar, order, 2> m_B;

    public:
        /**
         * @brief Construct an extended state observer
         *
         * @param gain rate of change in the (order - 1)-th state (velocity for 2nd order, acceleration for 3rd order) per a unit of input, i.e. the rate of change in the (order - 1)-th state divided by the input
         * @param convergence convergence rate of the estimated state to the actual state
         * @param State current state
         */
        constexpr ESO(Scalar gain, Scalar convergence, State state) noexcept
            : m_state{std::move(state)},
              m_A{A - L(convergence) * C},
              m_B{[convergence, gain]()
                  {
                      auto const &B0(B(gain)), &L0{L(convergence)};
                      decltype(m_B) ret{};
                      for (std::size_t ii{}; ii < order; ++ii)
                      {
                          ret[ii] = {B0(ii), L0(ii)};
                      }
                      return ret;
                  }()}
        {
        }

        /**
         * @brief Update the estimated state
         *
         * @param input current input
         * @param output current output
         * @param dt time since last update
         */
        constexpr auto update(Scalar input, Scalar output, Scalar dt) noexcept
        {
            m_state += dt * (m_A * m_state + m_B * math::Vector<Scalar, 2>{
                                                       input, output - m_state(0)});
        }
    };

    /**
     * @brief 2nd order extended state observer using `float`
     */
    using ESO2f = ESO<float, 2>;

    /**
     * @brief 3rd order extended state observer using `float`
     */
    using ESO3f = ESO<float, 3>;

    /**
     * @brief 2nd order extended state observer using `double`
     */
    using ESO2d = ESO<double, 2>;

    /**
     * @brief 3rd order extended state observer using `double`
     */
    using ESO3d = ESO<double, 3>;

    /**
     * @brief Controller implementing active disturbance rejection control (ADRC)
     *
     * @tparam Scalar type of scalar
     * @tparam order system order
     */
    template <typename Scalar, std::size_t order>
    class ADRC
    {
    public:
        /**
         * @brief Extended state observer type
         */
        using Observer = ESO<Scalar, order>;

        /**
         * @brief State type
         */
        using State = typename Observer::State;

        /**
         * @brief Ratio of observer convergence over controller convergence
         */
        constexpr static Scalar const observer_convergence_factor{10};

        /**
         * @brief The extended state observer
         */
        Observer m_observer;

    private:
        /**
         * @brief Rate of change in the (order - 1)-th state (velocity for 2nd order, acceleration for 3rd order) per a unit of input, i.e. the rate of change in the (order - 1)-th state divided by the input
         */
        Scalar m_gain;

        /**
         * @brief Controller constants determining the convergence rate to the desired state
         */
        math::RowVector<Scalar, order> m_K;

    public:
        /**
         * @brief Construct an ADRC controller
         *
         * @param gain rate of change in the (order - 1)-th state (velocity for 2nd order, acceleration for 3rd order) per a unit of input, i.e. the rate of change in the (order - 1)-th state divided by the input
         * @param convergence convergence rate of the state to the desired state
         * @param state current state
         */
        constexpr ADRC(Scalar gain, Scalar convergence, State state) noexcept
            : m_observer{gain,
                         convergence * observer_convergence_factor,
                         std::move(state)},
              m_gain{gain},
              m_K{[convergence]()
                  {
                      decltype(m_K) ret{};
                      for (std::size_t ii{}; ii < order; ++ii)
                      {
                          // (s+w)^(n-1)
                          // n=3, (s+w)^2=w^2+2ws+s^2
                          ret(ii) = math::tbinom(static_cast<Scalar>(order - 1), static_cast<Scalar>(order - 1 - ii)) * std::pow(convergence, static_cast<Scalar>(order - 1 - ii));
                      }
                      return ret;
                  }()}
        {
        }

        /**
         * @brief Recommend an input to reach the desired state
         *
         * @param target desired state
         * @return a recommended input
         */
        constexpr auto control [[nodiscard]] (Scalar target) const noexcept
        {
            auto const &state{m_observer.m_state};
            return (m_K * (State{target} - state))(0) / m_gain;
        }

        /**
         * @brief Update the extended state observer and recommend an input to reach the desired state
         *
         * @param target desired state
         * @param input current input
         * @param output current output
         * @param dt time since last update
         * @return a recommended input
         */
        constexpr auto update(Scalar target,
                              Scalar input,
                              Scalar output,
                              Scalar dt) noexcept
        {
            m_observer.update(input, output, dt);
            return control(target / static_cast<Scalar>(2));
        }
    };

    /**
     * @brief 2nd order ADRC using `float`
     */
    using ADRC2f = ADRC<float, 2>;

    /**
     * @brief 3rd order ADRC using `float`
     */
    using ADRC3f = ADRC<float, 3>;

    /**
     * @brief 2nd order ADRC using `double`
     */
    using ADRC2d = ADRC<double, 2>;

    /**
     * @brief 3rd order ADRC using `double`
     */
    using ADRC3d = ADRC<double, 3>;
} // namespace control
