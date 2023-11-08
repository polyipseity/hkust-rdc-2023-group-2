#include "mains.hpp"

#include <array>
#include <cstdlib>
#include <tuple>
#include <type_traits>

#include "can.h"
#include "communication.hpp"
#include "gpio.hpp"
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
  constexpr auto const auto_robot_translation_velocity{1.8};
  constexpr auto const auto_robot_translation_backward_velocity{.8};
  constexpr auto const auto_robot_rotation_velocity{math::tau};
  constexpr auto const auto_robot_calibrate_angular_velocity{math::tau / 32.};
  constexpr auto const auto_robot_line_tracker_correction_time{.1};
  constexpr auto const task_robot_translation_velocity{1.8};    // todo: use real values
  constexpr auto const task_robot_rotation_velocity{math::tau}; // todo: use real values
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

    Time time{};
    while (true)
    {
      auto const dt{time.update()};

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
    }
  }

  auto test_motor [[noreturn]] (RobotType type, Motor motor_handle) noexcept -> void
  {
    CANMotors<1> motors{{motor_handle}};
    std::array<control::ADRC2d, 1> motor_adrcs{
        type == RobotType::TASK ? new_motor_ADRC_task(motors[0]) : new_motor_ADRC_auto(motors[0]),
    };
    PositionADRC pos_adrc{0., 0.};

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

    CANMotors<2> motors_r{{CAN1_MOTOR1, CAN1_MOTOR0},
                          {false, true}};
    std::array<control::ADRC2d, 2> motor_adrcs{
        new_motor_ADRC_auto(motors_r[0]),
        new_motor_ADRC_auto(motors_r[1]),
    };
    AutoRobotADRC move_adrc{0., 0., {motors_r[0].getVelocity(), motors_r[1].getVelocity()}};

    auto dt{0.};
    auto active{true};
    double target_pos{}, target_rot{};

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
                       target_pos += auto_robot_translation_velocity * dt;
                     });
    commander.handle('s',
                     [&dt, &target_pos](typename decltype(commander)::ParamType const &)
                     {
                       target_pos += -auto_robot_translation_backward_velocity * dt;
                     });
    commander.handle('q',
                     [&dt, &target_rot](typename decltype(commander)::ParamType const &)
                     {
                       target_rot += auto_robot_rotation_velocity * dt;
                     });
    commander.handle('e',
                     [&dt, &target_rot](typename decltype(commander)::ParamType const &)
                     {
                       target_rot += -auto_robot_rotation_velocity * dt;
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
      auto const [v_l, v_r]{move_adrc.update(target_pos, target_rot, {motors[0].getVelocity(), motors[1].getVelocity()}, dt)};
      update_motor_velocity(motors[0], motor_adrcs[0], active * v_l, dt);
      update_motor_velocity(motors[1], motor_adrcs[1], active * v_r, dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "pos: %.2f", move_adrc.m_position);
        tft_prints(0, 1, "pos_t: %.2f", target_pos);
        tft_prints(0, 2, "rot: %.2f", math::rotation_matrix2_angle(move_adrc.m_rotation));
        tft_prints(0, 3, "rot_t: %.2f", target_rot);
        tft_prints(0, 4, "v: %.2f, %.2f", motors[0].getVelocity(), motors[1].getVelocity());
        tft_prints(0, 5, "v_t: %.2f, %.2f", v_l, v_r);
      }
    }
  }
}

namespace main
{
  auto auto_robot [[noreturn]] () -> void
  {
    CANMotors<2> motors_r{{CAN1_MOTOR1, CAN1_MOTOR0},
                          {false, true}};
    std::array<control::ADRC2d, 2> motor_adrcs{
        new_motor_ADRC_auto(motors_r[0]),
        new_motor_ADRC_auto(motors_r[1]),
    };
    AutoRobotADRC move_adrc{0., 0., {motors_r[0].getVelocity(), motors_r[1].getVelocity()}};
    GPIO line_sensor_left{CAM_D1_GPIO_Port, CAM_D1_Pin}, line_sensor_right{CAM_D3_GPIO_Port, CAM_D3_Pin};

    auto dt{0.};
    auto active{true};
    double target_pos{}, target_rot{};

    Commander<2> commander{};
    Receiver<16, false> receiver{huart1, .05};
    commander.handle('x',
                     [&active, &receiver](typename decltype(commander)::ParamType const &)
                     {
                       receiver.invalidate();
                       active = !active;
                     });

    HAL_Delay(1000);

    Time time{};
    double rot_correct_to_right{}, rot_correct_to_left{};
    while (rot_correct_to_right == 0. || rot_correct_to_left == 0.)
    {
      dt = time.update();

      if (rot_correct_to_right == 0.)
      {
        if (line_sensor_right.read())
        {
          rot_correct_to_right = -target_rot;
        }
        target_rot += auto_robot_calibrate_angular_velocity * dt;
      }
      else
      {
        if (line_sensor_left.read())
        {
          rot_correct_to_left = -target_rot;
        }
        target_rot -= auto_robot_calibrate_angular_velocity * dt;
      }

      CANMotorsControl<2> motors{motors_r};
      auto const [v_l, v_r]{move_adrc.update(target_pos, target_rot, {motors[0].getVelocity(), motors[1].getVelocity()}, dt)};
      update_motor_velocity(motors[0], motor_adrcs[0], v_l, dt);
      update_motor_velocity(motors[1], motor_adrcs[1], v_r, dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "pos: %.2f", move_adrc.m_position);
        tft_prints(0, 1, "pos_t: %.2f", target_pos);
        tft_prints(0, 2, "rot: %.2f", math::rotation_matrix2_angle(move_adrc.m_rotation));
        tft_prints(0, 3, "rot_t: %.2f", target_rot);
        tft_prints(0, 4, "v: %.2f, %.2f", motors[0].getVelocity(), motors[1].getVelocity());
        tft_prints(0, 5, "v_t: %.2f, %.2f", v_l, v_r);
        tft_prints(0, 6, "sensor: %d, %d", line_sensor_left.read(), line_sensor_right.read());
        tft_prints(0, 7, "calibrating");
      }
    }

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
      else
      {
        auto const line_left{line_sensor_left.read()}, line_right{line_sensor_right.read()};
        if (line_left && line_right)
        {
          active = false;
        }
        else if (line_left)
        {
          target_rot += rot_correct_to_left * dt / auto_robot_line_tracker_correction_time;
        }
        else if (line_right)
        {
          target_rot += rot_correct_to_right * dt / auto_robot_line_tracker_correction_time;
        }
        target_pos += auto_robot_translation_velocity / 4. * dt;
      }

      CANMotorsControl<2> motors{motors_r};
      auto const [v_l, v_r]{move_adrc.update(target_pos, target_rot, {motors[0].getVelocity(), motors[1].getVelocity()}, dt)};
      update_motor_velocity(motors[0], motor_adrcs[0], active * v_l, dt);
      update_motor_velocity(motors[1], motor_adrcs[1], active * v_r, dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "pos: %.2f", move_adrc.m_position);
        tft_prints(0, 1, "pos_t: %.2f", target_pos);
        tft_prints(0, 2, "rot: %.2f", math::rotation_matrix2_angle(move_adrc.m_rotation));
        tft_prints(0, 3, "rot_t: %.2f", target_rot);
        tft_prints(0, 4, "v: %.2f, %.2f", motors[0].getVelocity(), motors[1].getVelocity());
        tft_prints(0, 5, "v_t: %.2f, %.2f", v_l, v_r);
        tft_prints(0, 6, "sensor: %d, %d", line_sensor_left.read(), line_sensor_right.read());
      }
    }
  }

  auto task_robot [[noreturn]] () -> void
  {
    CANMotors<4> motors_r{{CAN1_MOTOR3, CAN1_MOTOR0, CAN2_MOTOR0, CAN2_MOTOR1},
                          {false, true, false, true}};
    std::array<control::ADRC2d, 4> motor_adrcs{
        new_motor_ADRC_task(motors_r[0]),
        new_motor_ADRC_task(motors_r[1]),
        new_motor_ADRC_task(motors_r[2]),
        new_motor_ADRC_task(motors_r[3]),
    };
    TaskRobotADRC move_adrc{{0., 0.}, 0., {motors_r[0].getVelocity(), motors_r[1].getVelocity(), motors_r[2].getVelocity(), motors_r[3].getVelocity()}};

    auto dt{0.};
    auto active{true};
    math::Vector<double, 2> target_pos{};
    double target_rot{};

    Commander<3> commander{};
    Receiver<16, false> receiver{huart1, .05};
    commander.handle('x',
                     [&active, &receiver](typename decltype(commander)::ParamType const &)
                     {
                       receiver.invalidate();
                       active = !active;
                     });
    auto wasd_command_capture{std::tie(dt, target_pos, target_rot)};
    commander.handle('w',
                     [&wasd_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{wasd_command_capture};
                       target_pos += math::rotation_matrix2(target_rot) * std::remove_reference_t<decltype(target_pos)>{0., task_robot_translation_velocity * dt};
                     });
    commander.handle('a',
                     [&wasd_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{wasd_command_capture};
                       target_pos += math::rotation_matrix2(target_rot) * std::remove_reference_t<decltype(target_pos)>{-task_robot_translation_velocity * dt, 0.};
                     });
    commander.handle('s',
                     [&wasd_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{wasd_command_capture};
                       target_pos += math::rotation_matrix2(target_rot) * std::remove_reference_t<decltype(target_pos)>{0., -task_robot_translation_velocity * dt};
                     });
    commander.handle('d',
                     [&wasd_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{wasd_command_capture};
                       target_pos += math::rotation_matrix2(target_rot) * std::remove_reference_t<decltype(target_pos)>{task_robot_translation_velocity * dt, 0.};
                     });
    commander.handle('q',
                     [&dt, &target_rot](typename decltype(commander)::ParamType const &)
                     {
                       target_rot += task_robot_rotation_velocity * dt;
                     });
    commander.handle('e',
                     [&dt, &target_rot](typename decltype(commander)::ParamType const &)
                     {
                       target_rot += -task_robot_rotation_velocity * dt;
                     });
    auto g_command_capture{std::tie(target_pos, target_rot, receiver)};
    commander.handle('g',
                     [&g_command_capture](typename decltype(commander)::ParamType const &param)
                     {
                       auto &[target_pos, target_rot, receiver]{g_command_capture};
                       receiver.invalidate();
                       auto const &[p_x, p_y, p_r]{param};
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

      CANMotorsControl<4> motors{motors_r};
      auto const [v_fl, v_fr, v_rl, v_rr]{(active * move_adrc.update(target_pos, target_rot, {motors[0].getVelocity(), motors[1].getVelocity(), motors[2].getVelocity(), motors[3].getVelocity()}, dt)).transpose()[0]};
      update_motor_velocity(motors[0], motor_adrcs[0], v_fl, dt);
      update_motor_velocity(motors[1], motor_adrcs[1], v_fr, dt);
      update_motor_velocity(motors[2], motor_adrcs[2], v_rl, dt);
      update_motor_velocity(motors[3], motor_adrcs[3], v_rr, dt);

      if (tft_update(tft_update_period))
      {
        tft_prints(0, 0, "pos: %.2f, %.2f", move_adrc.m_position(0), move_adrc.m_position(1));
        tft_prints(0, 1, "pos_t: %.2f, %.2f", target_pos(0), target_pos(1));
        tft_prints(0, 2, "rot: %.2f", math::rotation_matrix2_angle(move_adrc.m_rotation));
        tft_prints(0, 3, "rot_t: %.2f", target_rot);
        tft_prints(0, 4, "v: %.2f, %.2f", motors[0].getVelocity(), motors[1].getVelocity());
        tft_prints(0, 5, "   %.2f, %.2f", motors[2].getVelocity(), motors[3].getVelocity());
        tft_prints(0, 6, "v_t: %.2f, %.2f", v_fl, v_fr);
        tft_prints(0, 7, "     %.2f, %.2f", v_rl, v_rr);
      }
    }
  }
}
