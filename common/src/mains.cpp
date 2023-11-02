#include "mains.hpp"

#include <array>

#include "can.h"
#include "lcd/lcd.h"
#include "main.h"
#include "motor.hpp"
#include "position.hpp"
#include "receiver.hpp"
#include "util/adrc.hpp"

namespace test
{
  auto find_motor_gain [[noreturn]] (Motor motor_handle) noexcept -> void
  {
    constexpr static auto const input{.5}, duration{1.};
    CANMotors<1> motors{{motor_handle}};
    auto const start_tick{HAL_GetTick()};
    while ((HAL_GetTick() - start_tick) / 1000. <= duration)
    {
      CANMotorsControl<1> motors_ctrl{motors};
      auto &motor{motors_ctrl[0]};
      motor.setInput(input);
    }
    auto velocity{0.};
    while (velocity == 0.)
    {
      CANMotorsControl<1> motors_ctrl{motors};
      auto &motor{motors_ctrl[0]};
      motor.setInput(0);
      velocity = motor.getVelocity();
    }
    while (true)
    {
      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "input: %.6f", input);
        tft_prints(0, 1, "duration: %.6f", duration);
        tft_prints(0, 2, "velocity: %.6f", velocity);
        tft_prints(0, 3, "gain: %.6f", velocity / input / duration / 2.);
      }
    }
  }

  auto test_motor [[noreturn]] (Motor motor_handle) noexcept -> void
  {
    CANMotors<1> motors{{motor_handle}};
    std::array<control::ADRC2d, 1> motor_adrcs{
        new_motor_ADRC_auto(motors[0]),
    };
    PositionADRC pos_adrc{0., 0.};

    auto last_tick{HAL_GetTick()};
    auto target_position{0.};
    while (true)
    {
      auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
      auto const dt{elapsed / 1000.};
      CANMotorsControl<1> motors_ctrl{motors};
      auto &motor{motors_ctrl[0]};

      if (!btn_read(BTN1))
      {
        target_position -= 100. * dt;
      }
      if (!btn_read(BTN2))
      {
        target_position += 100. * dt;
      }

      update_motor_velocity(motor, motor_adrcs[0], pos_adrc.update(target_position, motor.getVelocity(), dt), dt);
      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "tick: %u", static_cast<unsigned int>(HAL_GetTick()));
        tft_prints(0, 1, "input: %.3f", motor.getInput());
        tft_prints(0, 2, "vel: %.3f", motor.getVelocity());
        tft_prints(0, 3, "pos: %.3f", pos_adrc.m_position);
        tft_prints(0, 4, "t pos: %.3f", target_position);
        tft_prints(0, 5, "diff: %.3f", pos_adrc.m_position - target_position);
      }

      last_tick = tick;
      HAL_Delay(1);
    }
  }

  auto test_uart [[noreturn]] (UART_HandleTypeDef &uart_handle) noexcept -> void
  {
    Receiver<1> receiver{uart_handle};
    while (true)
    {
      auto const update_tft{tft_update(tft_update_period)};

      auto const received{receiver.update()};
      if (received)
      {
        auto const &received2{*received};
        HAL_UART_Transmit(&uart_handle, std::data(received2), std::size(received2), HAL_MAX_DELAY);
        if (update_tft)
        {
          tft_prints(0, 0, "received: %u", received2[0]);
        }
      }
    }
  }
}
