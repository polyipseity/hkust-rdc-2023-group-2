#include "motor.hpp"

#include <cmath>

#include "can.h"

namespace
{
    constexpr static auto const input_noise_threshold{.25};
}

CANMotor::CANMotor(decltype(m_handle) handle)
    : m_handle{handle}
{
}
auto CANMotor::setInput(double input) noexcept -> void
{
    set_motor_current(m_handle, input);
}
auto CANMotor::getInput [[nodiscard]] () const noexcept -> double
{
    auto const ret{get_motor_feedback(m_handle).actual_current};
    return std::abs(ret) <= input_noise_threshold ? 0. : ret;
}
auto CANMotor::getPosition [[nodiscard]] () const noexcept -> double
{
    return get_motor_feedback(m_handle).encoder / 8192.;
}
auto CANMotor::getVelocity [[nodiscard]] () const noexcept -> double
{
    return get_motor_feedback(m_handle).vel_rpm / 1024.;
}
auto CANMotor::getTemperature [[nodiscard]] () const noexcept -> decltype(MotorStats::temperature)
{
    return get_motor_feedback(m_handle).temperature;
}

auto new_motor_ADRC_auto [[nodiscard]] (CANMotor const &motor, double convergence, double gain) noexcept -> control::ADRC2d
{
    return {gain, convergence, {motor.getVelocity()}};
}
auto new_motor_ADRC_mec [[nodiscard]] (CANMotor const &motor, double convergence, double gain) noexcept -> control::ADRC2d
{
    return {gain, convergence, {motor.getVelocity()}};
}
