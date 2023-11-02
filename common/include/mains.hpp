#pragma once

#include "can.h"
#include "main.h"

constexpr auto const tft_update_period{10};

namespace test
{
  auto find_motor_gain [[noreturn]] (Motor motor_handle) noexcept -> void;
  auto test_motor [[noreturn]] (Motor motor_handle) noexcept -> void;
  auto test_uart [[noreturn]] (UART_HandleTypeDef &uart_handle) noexcept -> void;
}
