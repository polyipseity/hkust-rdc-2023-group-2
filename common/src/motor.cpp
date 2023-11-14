#include "motor.hpp"

#include "can.h"

auto new_motor_ADRC_auto [[nodiscard]] (CANMotor const &motor, double convergence, double gain) noexcept -> control::ADRC2d
{
    return {gain, convergence, {CANMotor_get_velocity(&motor)}};
}

auto new_motor_ADRC_task [[nodiscard]] (CANMotor const &motor, double convergence, double gain) noexcept -> control::ADRC2d
{
    return {gain, convergence, {CANMotor_get_velocity(&motor)}};
}
