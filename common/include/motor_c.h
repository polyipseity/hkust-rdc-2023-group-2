#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "can.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief A CAN motor.
     */
    typedef struct CANMotor
    {
        /**
         * @brief The motor handle corresponding to the motor hardware
         */
        Motor m_handle;

        /**
         * @brief Motor direction factor
         */
        int m_factor;
    } CANMotor;

    /**
     * @brief Construct a new `CANMotor` object
     *
     * @param handle motor handle
     * @param reversed reverse motor direction
     */
    CANMotor CANMotor_new(Motor handle, bool reversed);

    /**
     * @brief Set the desired current to the motor
     *
     * @param self `CANMotor`
     * @param input desired current
     */
    void CANMotor_set_input(CANMotor *self, double input);

    /**
     * @brief Get the actual current to the motor
     *
     * @param self `CANMotor`
     * @return actual current
     */
    double CANMotor_get_input(CANMotor const *self);

    /**
     * @brief Get the bearing position of the motor
     *
     * @param self `CANMotor`
     * @return bearing position in the range [0, 1]
     */
    double CANMotor_get_position(CANMotor const *self);

    /**
     * @brief Get the rotation velocity of the motor bearing
     *
     * @param self `CANMotor`
     * @return rotation velocity in revolutions per second
     */
    double CANMotor_get_velocity(CANMotor const *self);

    /**
     * @brief Get the temperature of the motor
     *
     * @param self `CANMotor`
     * @return temperature
     */
    uint8_t CANMotor_get_temperature(CANMotor const *self);

#ifdef __cplusplus
}
#endif
