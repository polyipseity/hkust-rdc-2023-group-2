#include "motor.hpp"

#include "can.h"

CANMotor::CANMotor(decltype(m_handle) handle)
    : m_handle{handle}
{
}
auto CANMotor::setInput(decltype(MotorStats::actual_current) input) noexcept -> void
{
    set_motor_current(m_handle, input);
}
auto CANMotor::getInput() const noexcept -> decltype(MotorStats::actual_current)
{
    return get_motor_feedback(m_handle).actual_current;
}
auto CANMotor::getPosition() const noexcept -> decltype(MotorStats::encoder)
{
    return get_motor_feedback(m_handle).encoder;
}
auto CANMotor::getVelocity() const noexcept -> decltype(MotorStats::vel_rpm)
{
    return get_motor_feedback(m_handle).vel_rpm;
}
auto CANMotor::getTemperature() const noexcept -> decltype(MotorStats::temperature)
{
    return get_motor_feedback(m_handle).temperature;
}
