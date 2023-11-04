#pragma once

#include "can.h"
#include "main.h"

constexpr auto const tft_update_period{10};

namespace test
{
  /**
   * @brief Test a motor to find its gain
   *
   * @param motor_handle the motor to test
   */
  auto find_motor_gain [[noreturn]] (Motor motor_handle) noexcept -> void;

  /**
   * @brief Test controlling a motor
   *
   * @param motor_handle the motor to test
   */
  auto test_motor [[noreturn]] (Motor motor_handle) noexcept -> void;

  /**
   * @brief Test UART by echoing back received data
   *
   * @param uart_handle the UART to test
   */
  auto test_uart [[noreturn]] (UART_HandleTypeDef &uart_handle) noexcept -> void;
}
