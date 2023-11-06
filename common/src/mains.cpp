#include "mains.hpp"

#include <array>
#include <cstdlib>
#include <tuple>

#include "can.h"
#include "communication.hpp"
#include "lcd/lcd.h"
#include "main.h"
#include "motor.hpp"
#include "transform.hpp"
#include "usart.h"
#include "util/adrc.hpp"
#include "util/math.hpp"

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
        tft_prints(0, 3, "gain: %.6f", velocity / input / duration);
      }
    }
  }

  auto test_motor_velocity [[noreturn]] (RobotType type, Motor motor_handle, double velocity) noexcept -> void
  {
    CANMotors<1> motors{{motor_handle}};
    std::array<control::ADRC2d, 1> motor_adrcs{
        type == RobotType::TASK ? new_motor_ADRC_task(motors[0]) : new_motor_ADRC_auto(motors[0]),
    };

    auto last_tick{HAL_GetTick()};
    while (true)
    {
      auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
      if (elapsed <= 0)
      {
        continue;
      }
      auto dt{elapsed / 1000.};

      CANMotorsControl<1> motors_ctrl{motors};
      auto &motor{motors_ctrl[0]};
      update_motor_velocity(motor, motor_adrcs[0], velocity, dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "in: %.6f", motor.getInput());
        tft_prints(0, 1, "v: %.6f", motor.getVelocity());
        tft_prints(0, 2, "v_t: %.6f", velocity);
        tft_prints(0, 3, "v_d: %.6f", velocity - motor.getVelocity());
      }

      last_tick = tick;
    }
  }

  auto test_motor [[noreturn]] (RobotType type, Motor motor_handle) noexcept -> void
  {
    CANMotors<1> motors{{motor_handle}};
    std::array<control::ADRC2d, 1> motor_adrcs{
        type == RobotType::TASK ? new_motor_ADRC_task(motors[0]) : new_motor_ADRC_auto(motors[0]),
    };
    PositionADRC pos_adrc{0., 0.};

    auto last_tick{HAL_GetTick()};
    auto target_position{0.};
    while (true)
    {
      auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
      if (elapsed <= 0)
      {
        continue;
      }
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
        tft_prints(0, 0, "tick: %u", (unsigned int){HAL_GetTick()});
        tft_prints(0, 1, "input: %.3f", motor.getInput());
        tft_prints(0, 2, "vel: %.3f", motor.getVelocity());
        tft_prints(0, 3, "pos: %.3f", pos_adrc.m_position);
        tft_prints(0, 4, "t pos: %.3f", target_position);
        tft_prints(0, 5, "diff: %.3f", pos_adrc.m_position - target_position);
      }

      last_tick = tick;
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

  auto test_auto_robot_movement [[noreturn]] () -> void
  {
    CANMotors<2> motors_r{{CAN1_MOTOR0, CAN1_MOTOR1},
                          {false, true}};
    std::array<control::ADRC2d, 2> motor_adrcs{
        new_motor_ADRC_auto(motors_r[0]),
        new_motor_ADRC_auto(motors_r[1]),
    };
    AutoRobotADRC move_adrc{{0., 0.}, 0., {motors_r[0].getVelocity(), motors_r[1].getVelocity()}};

    math::Vector<double, 2> target_pos{};
    auto last_tick{HAL_GetTick()};
    auto active{true};

    auto dt{0.};
    Commander<2> commander{};
    Receiver<16, false> receiver{huart1, .05};
    commander.handle('x',
                     [&active, &receiver](typename Commander<2>::ParamType const &)
                     {
                       receiver.invalidate();
                       active = !active;
                     });
    commander.handle('w', [&dt, &target_pos](typename Commander<2>::ParamType const &)
                     { target_pos += {0., 1. * dt}; });
    commander.handle('a', [&dt, &target_pos](typename Commander<2>::ParamType const &)
                     { target_pos += {-1. * dt, 0.}; });
    commander.handle('s', [&dt, &target_pos](typename Commander<2>::ParamType const &)
                     { target_pos += {0., -1. * dt}; });
    commander.handle('d', [&dt, &target_pos](typename Commander<2>::ParamType const &)
                     { target_pos += {1. * dt, 0.}; });
    commander.handle('g',
                     [&target_pos, &receiver](typename Commander<2>::ParamType const &param)
                     {
                       receiver.invalidate();
                       auto const &[p_x, p_y]{param};
                       if (std::get<0>(p_x) == 0 || std::get<0>(p_y) == 0)
                       {
                         return;
                       }
                       char *end{};
                       auto const xx{std::strtod(std::get<1>(p_x), &end)};
                       if (std::get<1>(p_x) == end)
                       {
                         return;
                       }
                       auto const yy{std::strtod(std::get<1>(p_y), &end)};
                       if (std::get<1>(p_y) == end)
                       {
                         return;
                       }
                       target_pos = {xx, yy};
                     });

    while (true)
    {
      auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
      if (elapsed <= 0)
      {
        continue;
      }
      dt = elapsed / 1000.;

      auto const received{receiver.update()};
      commander.dispatch(std::get<1>(received), std::get<0>(received));

      CANMotorsControl<2> motors{motors_r};
      if (!active)
      {
        target_pos = move_adrc.m_position;
      }
      auto const [v_l, v_r] = move_adrc.update(target_pos, {motors[0].getVelocity(), motors[1].getVelocity()}, dt);
      update_motor_velocity(motors[0], motor_adrcs[0], active * v_l, dt);
      update_motor_velocity(motors[1], motor_adrcs[1], active * v_r, dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "pos: %.2f, %.2f", move_adrc.m_position(0), move_adrc.m_position(1));
        tft_prints(0, 1, "rot: %.2f", math::rotation_matrix2_angle(move_adrc.m_rotation));
        tft_prints(0, 2, "v: %.2f, %.2f", motors[0].getVelocity(), motors[1].getVelocity());
        tft_prints(0, 3, "d_v: %.2f, %.2f", v_l, v_r);
      }

      last_tick = tick;
    }
  }
}
