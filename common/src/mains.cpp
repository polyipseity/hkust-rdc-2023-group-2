#include "mains.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <tuple>
#include <type_traits>
#include <utility>

#include "can.h"
#include "communication.hpp"
#include "gpio_ext.hpp"
#include "lcd/lcd.h"
#include "main.h"
#include "motor.hpp"
#include "transform.hpp"
#include "usart.h"
#include "util.hpp"
#include "util/adrc.hpp"
#include "util/math.hpp"

namespace
{
  constexpr auto const motor_test_velocity{1000.};
  constexpr auto const test_auto_robot_translation_velocity{1.8};
  constexpr auto const test_auto_robot_translation_backward_velocity{.8};
  constexpr auto const test_auto_robot_rotation_velocity{math::tau};
}

namespace test
{
  auto find_motor_gain [[noreturn]] (Motor motor_handle) noexcept -> void
  {
    constexpr static auto const input{1.}, duration{.5};
    CANMotors<1> motors{{motor_handle}};
    auto const start_tick{HAL_GetTick()};
    while ((HAL_GetTick() - start_tick) / 1000. <= duration)
    {
      CANMotorsControl<1> motors_ctrl{motors};
      auto &motor{motors_ctrl[0]};
      motor.set_input(input);
    }
    auto velocity{0.};
    while (velocity == 0.)
    {
      CANMotorsControl<1> motors_ctrl{motors};
      auto &motor{motors_ctrl[0]};
      motor.set_input(0);
      velocity = motor.get_velocity();
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

    Time time{};
    while (true)
    {
      auto const dt{time.update()};

      CANMotorsControl<1> motors_ctrl{motors};
      auto &motor{motors_ctrl[0]};
      update_motor_velocity(motor, motor_adrcs[0], velocity, dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "in: %.6f", motor.get_input());
        tft_prints(0, 1, "v: %.6f", motor.get_velocity());
        tft_prints(0, 2, "v_t: %.6f", velocity);
        tft_prints(0, 3, "v_d: %.6f", velocity - motor.get_velocity());
      }
    }
  }

  auto test_motor [[noreturn]] (RobotType type, Motor motor_handle) noexcept -> void
  {
    CANMotors<1> motors{{motor_handle}};
    std::array<control::ADRC2d, 1> motor_adrcs{
        type == RobotType::TASK ? new_motor_ADRC_task(motors[0]) : new_motor_ADRC_auto(motors[0]),
    };
    PositionADRC pos_adrc{0., motors[0].get_velocity()};

    Time time{};
    auto target_position{0.};
    while (true)
    {
      auto const dt{time.update()};
      CANMotorsControl<1> motors_ctrl{motors};
      auto &motor{motors_ctrl[0]};

      if (!btn_read(BTN1))
      {
        target_position -= motor_test_velocity * dt;
      }
      if (!btn_read(BTN2))
      {
        target_position += motor_test_velocity * dt;
      }

      update_motor_velocity(motor, motor_adrcs[0], pos_adrc.update(target_position, motor.get_velocity(), dt), dt);
      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "tick: %u", (unsigned int){HAL_GetTick()});
        tft_prints(0, 1, "input: %.3f", motor.get_input());
        tft_prints(0, 2, "vel: %.3f", motor.get_velocity());
        tft_prints(0, 3, "pos: %.3f", pos_adrc.m_position);
        tft_prints(0, 4, "t pos: %.3f", target_position);
        tft_prints(0, 5, "diff: %.3f", pos_adrc.m_position - target_position);
      }
    }
  }

  auto test_uart [[noreturn]] (UART_HandleTypeDef &uart_handle) noexcept -> void
  {
    Receiver<CHAR_MAX_X_VERTICAL * CHAR_MAX_Y_VERTICAL, false> receiver{uart_handle, .025};
    while (true)
    {
      auto const update_tft{tft_update(tft_update_period)};
      auto const [receive_size, received]{receiver.update()};
      if (update_tft)
      {
        for (std::size_t ii{}; ii < receive_size; ++ii)
        {
          tft_prints(ii % CHAR_MAX_X_VERTICAL, ii / CHAR_MAX_X_VERTICAL, "%c", received[ii]);
        }
      }
    }
  }

  auto test_auto_robot_movement [[noreturn]] () -> void
  {
    CANMotors<2> motors_r{{CAN1_MOTOR1, CAN1_MOTOR0},
                          {false, true}};
    std::array<control::ADRC2d, 2> motor_adrcs{
        new_motor_ADRC_auto(motors_r[0]),
        new_motor_ADRC_auto(motors_r[1]),
    };
    AutoRobotADRC move_adrc{0., math::tau / 4., {motors_r[0].get_velocity(), motors_r[1].get_velocity()}};

    auto dt{0.};
    auto active{true};
    double target_pos{}, target_rot{math::tau / 4.};

    Commander<2> commander{};
    Receiver<16, false> receiver{huart1, .05};
    commander.handle('x',
                     [&active, &receiver](typename decltype(commander)::ParamType const &)
                     {
                       receiver.invalidate();
                       active = !active;
                     });
    commander.handle('w',
                     [&dt, &target_pos](typename decltype(commander)::ParamType const &)
                     {
                       target_pos += test_auto_robot_translation_velocity * dt;
                     });
    commander.handle('s',
                     [&dt, &target_pos](typename decltype(commander)::ParamType const &)
                     {
                       target_pos += -test_auto_robot_translation_backward_velocity * dt;
                     });
    commander.handle('q',
                     [&dt, &target_rot](typename decltype(commander)::ParamType const &)
                     {
                       target_rot += test_auto_robot_rotation_velocity * dt;
                     });
    commander.handle('e',
                     [&dt, &target_rot](typename decltype(commander)::ParamType const &)
                     {
                       target_rot += -test_auto_robot_rotation_velocity * dt;
                     });
    auto g_command_capture{std::tie(target_pos, target_rot, receiver)};
    commander.handle('g',
                     [&g_command_capture](typename decltype(commander)::ParamType const &param)
                     {
                       auto &[target_pos, target_rot, receiver]{g_command_capture};
                       receiver.invalidate();
                       auto const &[p_x, p_r]{param};
                       if (std::get<0>(p_x) == 0)
                       {
                         return;
                       }
                       char *end{};
                       auto const xx{std::strtod(std::get<1>(p_x), &end)};
                       if (std::get<1>(p_x) == end)
                       {
                         return;
                       }
                       target_pos = xx;
                       if (std::get<0>(p_r) == 0)
                       {
                         return;
                       }
                       auto const rot{std::strtod(std::get<1>(p_r), &end)};
                       if (std::get<1>(p_r) == end)
                       {
                         return;
                       }
                       target_rot = rot;
                     });

    Time time{};
    while (true)
    {
      dt = time.update();

      auto const [received_size, received]{receiver.update()};
      commander.dispatch(received, received_size);

      if (!active)
      {
        target_pos = move_adrc.m_position;
        target_rot = math::rotation_matrix2_angle(move_adrc.m_rotation);
      }

      CANMotorsControl<2> motors{motors_r};
      auto const [v_l, v_r]{move_adrc.update(target_pos, target_rot, {motors[0].get_velocity(), motors[1].get_velocity()}, dt, {test_auto_robot_translation_velocity, test_auto_robot_rotation_velocity})};
      update_motor_velocity(motors[0], motor_adrcs[0], active * v_l, dt);
      update_motor_velocity(motors[1], motor_adrcs[1], active * v_r, dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "pos: %.2f", move_adrc.m_position);
        tft_prints(0, 1, "pos_t: %.2f", target_pos);
        tft_prints(0, 2, "rot: %.2f", math::rotation_matrix2_angle(move_adrc.m_rotation));
        tft_prints(0, 3, "rot_t: %.2f", target_rot);
        tft_prints(0, 4, "v: %.2f, %.2f", motors[0].get_velocity(), motors[1].get_velocity());
        tft_prints(0, 5, "v_t: %.2f, %.2f", v_l, v_r);
      }
    }
  }

  auto read_motor_velocities [[noreturn]] () noexcept -> void
  {
    CANMotors<MAX_NUM_OF_MOTORS> motors_r{{
        CAN1_MOTOR0,
        CAN1_MOTOR1,
        CAN1_MOTOR2,
        CAN1_MOTOR3,
        CAN1_MOTOR4,
        CAN1_MOTOR5,
        CAN1_MOTOR6,
        CAN1_MOTOR7,
        CAN2_MOTOR0,
        CAN2_MOTOR1,
        CAN2_MOTOR2,
        CAN2_MOTOR3,
        CAN2_MOTOR4,
        CAN2_MOTOR5,
        CAN2_MOTOR6,
        CAN2_MOTOR7,
    }};
    while (true)
    {
      CANMotorsControl<MAX_NUM_OF_MOTORS> motors{motors_r};
      if (tft_update(tft_update_period))
      {
        for (std::size_t ii{}; ii < MAX_NUM_OF_MOTORS; ++ii)
        {
          tft_prints(CHAR_MAX_X_VERTICAL / 2 * (ii % 2), ii / 2, "%.3f", motors[ii].get_velocity());
        }
      }
    }
  }

  auto test_auto_robot_thrower [[noreturn]] (Motor motor_handle) noexcept -> void
  {
    CANMotors<1> motors_r{{motor_handle}};
    std::array<control::ADRC2d, 1> motor_adrcs{
        new_motor_ADRC_auto(motors_r[0], .5, 8.),
    };
    PositionADRC thrower_adrc{0., motors_r[0].get_velocity(), math::tau * .1 * 8.5 / 30.};

    Time time{};
    double thrower_rot{math::tau / 2.};
    while (true)
    {
      auto const dt{time.update()};
      CANMotorsControl<1> motors{motors_r};

      auto const thrower_v{thrower_adrc.update(thrower_rot, motors[0].get_velocity(), dt)};
      update_motor_velocity(motors[0], motor_adrcs[0], std::copysign(std::min(math::tau / thrower_adrc.m_gain, std::abs(thrower_v)), thrower_v), dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "pos: %.3f", thrower_adrc.m_position);
        tft_prints(0, 1, "vel: %.3f", thrower_adrc.m_velocity / thrower_adrc.m_gain);
      }
    }
  }
}
