#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "can.h"
#include "util/adrc.hpp"

/**
 * @brief A CAN motor.
 */
class CANMotor
{
    /**
     * @brief The motor handle corresponding to the motor hardware
     */
    Motor m_handle;

    /**
     * @brief Motor direction factor
     */
    int m_factor;

public:
    /**
     * @brief Construct a new `CANMotor` object
     *
     * @param handle motor handle
     * @param reversed reverse motor direction
     */
    explicit CANMotor(decltype(m_handle) handle, bool reversed = false) noexcept;

    /**
     * @brief Set the desired current to the motor
     *
     * @param input desired current
     */
    auto setInput(double input) noexcept -> void;

    /**
     * @brief Get the actual current to the motor
     *
     * @return actual current
     */
    auto getInput [[nodiscard]] () const noexcept -> double;

    /**
     * @brief Get the bearing position of the motor
     *
     * @return bearing position in the range [0, 1]
     */
    auto getPosition [[nodiscard]] () const noexcept -> double;

    /**
     * @brief Get the rotation velocity of the motor
     *
     * @return rotation velocity in revolutions per second
     */
    auto getVelocity [[nodiscard]] () const noexcept -> double;

    /**
     * @brief Get the temperature of the motor
     *
     * @return temperature
     */
    auto getTemperature [[nodiscard]] () const noexcept -> decltype(MotorStats::temperature);

    constexpr CANMotor(CANMotor const &) noexcept = delete;

    constexpr CANMotor(CANMotor &&right) noexcept = default;

    constexpr auto operator=(CANMotor const &) noexcept -> CANMotor & = delete;

    constexpr auto operator=(CANMotor &&right) noexcept -> CANMotor & = default;
};

/**
 * @brief Set of CAN motors
 *
 * @tparam size_ number of motors
 */
template <std::size_t size_>
class CANMotors
{
private:
    /**
     * @brief Number of motors
     */
    constexpr static auto const size{size_};

    template <std::size_t size>
    friend class CANMotorsControl;

public:
    /**
     * @brief CAN motors
     */
    std::array<CANMotor, size> m_handles;

public:
    /**
     * @brief Construct a new `CANMotors` object
     *
     * @param handles an array of CAN motors
     * @param reversed reverse motor direction
     */
    constexpr explicit CANMotors(std::array<Motor, size> const &handles,
                                 std::array<bool, size> const &reversed = {}) noexcept
        : m_handles{[&handles, &reversed]<std::size_t... indices>(std::index_sequence<indices...>)
                    {
                        return decltype(m_handles){CANMotor{handles[indices], reversed[indices]}...};
                    }(std::make_index_sequence<size>{})}
    {
        can_init();
    }

    /**
     * @brief Get a read-only motor by index
     *
     * @param index index of the motor
     * @return a read-only motor
     */
    constexpr auto const &operator[] [[nodiscard]] (decltype(size) index) const noexcept
    {
        return m_handles[index];
    }

    constexpr CANMotors(CANMotors const &) noexcept = delete;

    constexpr CANMotors(CANMotors &&right) noexcept = default;

    constexpr auto operator=(CANMotors const &) noexcept -> CANMotors & = delete;

    constexpr auto operator=(CANMotors &&right) noexcept -> CANMotors & = default;
};

/**
 * @brief Controller for a set of CAN motors
 *
 * @tparam size_ number of motors
 */
template <std::size_t size_>
class CANMotorsControl
{
public:
    /**
     * @brief Number of motors
     */
    constexpr static auto const size{size_};

private:
    /**
     * @brief The set of CAN motors
     */
    CANMotors<size> *m_delegate;

public:
    /**
     * @brief Construct a new `CANMotorsControl` object
     *
     * @param handles a set of CAN motors
     */
    constexpr explicit CANMotorsControl(decltype(*m_delegate) const &delegate) noexcept
        : m_delegate{&delegate}
    {
    }

    /**
     * @brief Swap this motor control with the right motor control
     *
     * @param right motor control in the right operand
     */
    auto swap(CANMotorsControl &right) noexcept
    {
        using std::swap;
        swap(m_delegate, right.m_delegate);
    }

    /**
     * @brief Swap two motor controls
     *
     * @param left motor control in the left operand
     * @param right motor control in the right operand
     */
    friend auto swap(CANMotorsControl &left, CANMotorsControl &right) noexcept
    {
        left.swap(right);
    }

    /**
     * @brief Destroy the `CANMotorsControl object` and transmit new inputs to the CAN motors
     */
    ~CANMotorsControl() noexcept
    {
        if (m_delegate)
        {
            can_ctrl_loop();
        }
    }

    /**
     * @brief Get a controllable motor by index
     *
     * @param index index of the motor
     * @return a controllable motor
     */
    constexpr auto &operator[] [[nodiscard]] (decltype(size) index) noexcept
    {
        return m_delegate->m_handles[index];
    }

    /**
     * @brief Get a read-only motor by index
     *
     * @param index index of the motor
     * @return a read-only motor
     */
    constexpr auto const &operator[] [[nodiscard]] (decltype(size) index) const noexcept
    {
        return (*m_delegate)[index];
    }

    constexpr CANMotorsControl(CANMotorsControl const &) noexcept = delete;

    constexpr CANMotorsControl(CANMotorsControl &&right) noexcept
        : m_delegate{std::move(right.m_delegate)}
    {
        right.m_delegate = nullptr;
    }

    constexpr auto operator=(CANMotorsControl const &) noexcept -> CANMotorsControl & = delete;

    constexpr auto operator=(CANMotorsControl &&right) noexcept -> CANMotorsControl &
    {
        CANMotorsControl{std::move(right)}.swap(*this);
        return *this;
    };
};

/**
 * @brief Construct a new controller for motors in the auto robot
 *
 * @param motor motor handle
 * @param convergence control reactiveness
 * @param gain acceleration per current
 * @return a controller for the motor
 */
auto new_motor_ADRC_auto [[nodiscard]] (CANMotor const &motor, double convergence = 1., double gain = 1. * 36.) noexcept -> control::ADRC2d; // todo: readjust

/**
 * @brief Construct a new controller for motors in the task robot
 *
 * @param motor motor handle
 * @param convergence control reactiveness
 * @param gain acceleration per current
 * @return a controller for the motor
 */
auto new_motor_ADRC_task [[nodiscard]] (CANMotor const &motor, double convergence = 4., double gain = 20.) noexcept -> control::ADRC2d;

/**
 * @brief Minimum motor velocity
 */
constexpr auto const minimum_motor_velocity{.3};

/**
 *
 * @brief Minimum desired motor velocity for movement
 */
constexpr auto const motor_velocity_threshold{minimum_motor_velocity / 2.};

/**
 * @brief Update the velocity of a motor using the specified controller
 *
 * @tparam Controller controller type
 * @param motor motor to control
 * @param controller controller for the motor
 * @param velocity desired rotation velocity
 * @param dt time since last update
 */
template <typename Controller>
constexpr auto update_motor_velocity(CANMotor &motor, Controller &controller, double velocity, double dt) noexcept
{
    velocity = std::abs(velocity) >= motor_velocity_threshold
                   ? std::copysign(std::max(minimum_motor_velocity, std::abs(velocity)), velocity)
                   : 0.;
    auto const input{controller.update(velocity, motor.getInput(), motor.getVelocity(), dt)};
    motor.setInput(input);
}
