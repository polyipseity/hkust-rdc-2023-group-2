#include "motor.hpp"

#include <cmath>

#include "can.h"

namespace
{
    constexpr static auto const input_noise_threshold{.25};
}

CANMotor::CANMotor(decltype(m_handle) handle, bool reversed)
    : m_handle{handle}, m_factor{reversed ? -1 : 1}
{
}

auto CANMotor::set_input(double input) noexcept -> void
{
    set_motor_current(m_handle, m_factor * input);
}

auto CANMotor::get_input [[nodiscard]] () const noexcept -> double
{
    auto const ret{m_factor * get_motor_feedback(m_handle).actual_current};
    return std::abs(ret) <= input_noise_threshold ? 0. : ret;
}

auto CANMotor::get_position [[nodiscard]] () const noexcept -> double
{
    auto const ret{m_factor * get_motor_feedback(m_handle).encoder / 8192.};
    return ret >= 0. ? ret : 1. + ret;
}

auto CANMotor::get_velocity [[nodiscard]] () const noexcept -> double
{
    return m_factor * get_motor_feedback(m_handle).vel_rpm / 60.;
}

auto CANMotor::get_temperature [[nodiscard]] () const noexcept -> decltype(MotorStats::temperature)
{
    return get_motor_feedback(m_handle).temperature;
}

auto new_motor_ADRC_auto [[nodiscard]] (CANMotor const &motor, double convergence, double gain) noexcept -> control::ADRC2d
{
    return {gain, convergence, {motor.get_velocity()}};
}

auto new_motor_ADRC_task [[nodiscard]] (CANMotor const &motor, double convergence, double gain) noexcept -> control::ADRC2d
{
    return {gain, convergence, {motor.get_velocity()}};
}
