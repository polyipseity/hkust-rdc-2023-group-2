#pragma once

#include "can.h"
#include "main.h"

/**
 * @brief Robot type
 */
enum class RobotType
{
  TASK,
  AUTO,
};

/**
 * @brief Minimum TFT update interval
 */
constexpr auto const tft_update_period{10};

namespace test
{
  /**
   * @brief Test a motor to find its gain
   *
   * @param motor_handle motor to test
   */
  auto find_motor_gain [[noreturn]] (Motor motor_handle) noexcept -> void;

  /**
   * @brief Test setting a motor to a certain velocity.
   *
   * @param RobotType robot type
   * @param motor_handle motor to test
   * @param velocity desired velocity
   */
  auto test_motor_velocity [[noreturn]] (RobotType type, Motor motor_handle, double velocity) noexcept -> void;

  /**
   * @brief Test controlling a motor
   *
   * @param RobotType robot type
   * @param motor_handle motor to test
   */
  auto test_motor [[noreturn]] (RobotType type, Motor motor_handle) noexcept -> void;

  /**
   * @brief Test UART by echoing back received data
   *
   * @param uart_handle the UART to test
   */
  auto test_uart [[noreturn]] (UART_HandleTypeDef &uart_handle) noexcept -> void;

  /**
   * @brief Test movement of the auto robot
   */
  auto test_auto_robot_movement [[noreturn]] () noexcept -> void;
}
