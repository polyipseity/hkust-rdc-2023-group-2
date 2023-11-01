#include "motor.hpp"

#include "can.h"

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
    return get_motor_feedback(m_handle).actual_current;
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
