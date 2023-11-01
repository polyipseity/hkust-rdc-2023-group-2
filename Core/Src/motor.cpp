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
auto CANMotor::setInput(float input) noexcept -> void
{
    set_motor_current(m_handle, input);
}
auto CANMotor::getInput() const noexcept -> float
{
    auto const ret{get_motor_feedback(m_handle).actual_current};
    return std::abs(ret) <= input_noise_threshold ? 0. : ret;
}
auto CANMotor::getPosition() const noexcept -> float
{
    return get_motor_feedback(m_handle).encoder / 8192.;
}
auto CANMotor::getVelocity() const noexcept -> float
{
    return get_motor_feedback(m_handle).vel_rpm / 1024.;
}
auto CANMotor::getTemperature() const noexcept -> decltype(MotorStats::temperature)
{
    return get_motor_feedback(m_handle).temperature;
}
