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
}

namespace main
{
  namespace
  {
    constexpr auto const auto_robot_line_sensor_reversed{false};
    constexpr auto const auto_robot_initial_delay{2.};
    constexpr auto const auto_robot_translation_velocity{.2};
    constexpr auto const auto_robot_translation_tolerance{.01};
    constexpr auto const auto_robot_rotation_velocity{math::tau / 16.};
    constexpr auto const auto_robot_rotation_tolerance{math::tau / 128.};

    constexpr auto const auto_robot_calibration_initial_translation{.1};
    constexpr auto const auto_robot_calibrate_angular_velocity{math::tau / 32.};
    constexpr auto const auto_robot_line_tracker_correction_time{.25};
    constexpr auto const auto_robot_line_tracker_delay_time{.1};

    constexpr auto const auto_robot_navigation_initial_translation{.15};
    constexpr auto const auto_robot_navigation_angular_velocity{math::tau / 64.};
    constexpr auto const auto_robot_navigation_approach_translation{.2};
    constexpr auto const auto_robot_line_sensor_filter_time{.1};

    constexpr auto const auto_robot_thrower_max_velocity{1.}; // For safety, do not remove
    constexpr auto const auto_robot_thrower_velocity{1.};
  }

  auto auto_robot [[noreturn]] () -> void
  {
    CANMotors<3> motors_r{{CAN1_MOTOR1, CAN1_MOTOR0, CAN2_MOTOR2},
                          {false, true}};
    std::array<control::ADRC2d, 3> motor_adrcs{
        new_motor_ADRC_auto(motors_r[0]),
        new_motor_ADRC_auto(motors_r[1]),
        new_motor_ADRC_auto(motors_r[2], .5, 8.5),
    };
    AutoRobotADRC move_adrc{0., math::tau / 4., {motors_r[0].get_velocity(), motors_r[1].get_velocity()}};
    PositionADRC thrower_adrc{0., motors_r[2].get_velocity(), .1 * 8.5 / 30.};
    GPIO line_sensor_left{CAM_D1_GPIO_Port, CAM_D1_Pin, auto_robot_line_sensor_reversed},
        line_sensor_right{CAM_D3_GPIO_Port, CAM_D3_Pin, auto_robot_line_sensor_reversed},
        line_sensor_mid{CAM_D5_GPIO_Port, CAM_D5_Pin, auto_robot_line_sensor_reversed};

    auto dt{0.};
    auto active{true};
    double target_pos{}, target_rot{math::tau / 4.}, thrower_rot{};
    std::optional<std::array<long, 2>> box_targets{};

    Commander<2> commander{};
    Receiver<16, false> receiver{huart1, .05};
    commander.handle('x',
                     [&active, &receiver](typename decltype(commander)::ParamType const &)
                     {
                       receiver.invalidate();
                       active = !active;
                     });
    commander.handle('t',
                     [&box_targets, &receiver](typename decltype(commander)::ParamType const &params)
                     {
                       receiver.invalidate();
                       if (box_targets)
                       {
                         return;
                       }
                       auto &&[param0, param1]{params};
                       char *end0{}, *end1{};
                       auto const tar0{std::strtol(std::get<1>(param0), &end0, 10)},
                           tar1{std::strtol(std::get<1>(param1), &end1, 10)};
                       if (std::get<1>(param0) == end0 || std::get<1>(param1) == end1)
                       {
                         return;
                       }
                       box_targets = {tar0, tar1}; // 1, 2, 3, 4
                     });

    Time time{};
    auto const input{[&]()
                     {
                       dt = time.update();
                       auto const [received_size, received]{receiver.update()};
                       commander.dispatch(received, received_size);
                     }};
    auto const output{[&](char const state[])
                      {
                        if (!active)
                        {
                          target_pos = move_adrc.m_position;
                          target_rot = math::rotation_matrix2_angle(move_adrc.m_rotation);
                        }
                        CANMotorsControl<3> motors{motors_r};
                        auto const [v_l, v_r]{move_adrc.update(target_pos, target_rot, {motors[0].get_velocity(), motors[1].get_velocity()}, dt, {auto_robot_translation_velocity, auto_robot_rotation_velocity})};
                        update_motor_velocity(motors[0], motor_adrcs[0], active * v_l, dt);
                        update_motor_velocity(motors[1], motor_adrcs[1], active * v_r, dt);
                        auto const thrower_v{thrower_adrc.update(thrower_rot, motors[2].get_velocity(), dt)};
                        update_motor_velocity(motors[2], motor_adrcs[2], active * std::copysign(std::min(auto_robot_thrower_max_velocity / thrower_adrc.m_gain, std::abs(thrower_v)), thrower_v), dt);

                        if (tft_update(tft_update_period))
                        {
                          tft_prints(0, 0, "pos: %.2f", move_adrc.m_position);
                          tft_prints(0, 1, "pos_t: %.2f", target_pos);
                          tft_prints(0, 2, "rot: %.2f", math::rotation_matrix2_angle(move_adrc.m_rotation));
                          tft_prints(0, 3, "rot_t: %.2f", target_rot);
                          tft_prints(0, 4, "v: %.2f, %.2f", motors[0].get_velocity(), motors[1].get_velocity());
                          tft_prints(0, 5, "v_t: %.2f, %.2f", v_l, v_r);
                          tft_prints(0, 6, "sensor: %d, %d, %d", line_sensor_left.read(), line_sensor_mid.read(), line_sensor_right.read());
                          tft_prints(0, 7, "%s", state);
                        }
                      }};

    // Initialization
    auto const initial_time{time.time()};
    while (time.time() - initial_time <= auto_robot_initial_delay)
    {
      input();
      output("initializing");
    }

    // Calibration
    target_pos += auto_robot_calibration_initial_translation;
    while (true)
    {
      input();
      if (std::abs(target_pos - move_adrc.m_position) <= auto_robot_translation_tolerance)
      {
        break;
      }
      output("discovering");
    }
    double rot_correct_to_right{}, rot_correct_to_left{};
    while (rot_correct_to_right == 0. || rot_correct_to_left == 0.)
    {
      input();
      if (rot_correct_to_right == 0.)
      {
        if (line_sensor_right.read())
        {
          rot_correct_to_right = -target_rot + math::tau / 4.;
        }
        target_rot += auto_robot_calibrate_angular_velocity * dt;
      }
      else
      {
        if (line_sensor_left.read())
        {
          rot_correct_to_left = -target_rot + math::tau / 4.;
        }
        target_rot -= auto_robot_calibrate_angular_velocity * dt;
      }
      output("calibrating");
    }

    // Movement
    auto track_line{[&, last_line_left{false}, last_line_right{false}, last_change{time.time()}](bool line_left, bool line_right) mutable
                    {
                      if (last_line_left != line_left)
                      {
                        last_line_left = line_left;
                        last_change = time.time();
                      }
                      if (last_line_right != line_right)
                      {
                        last_line_right = line_right;
                        last_change = time.time();
                      }
                      if (time.time() - last_change <= auto_robot_line_tracker_delay_time)
                      {
                        return;
                      }
                      if (line_left)
                      {
                        target_rot += rot_correct_to_left * dt / auto_robot_line_tracker_correction_time;
                      }
                      if (line_right)
                      {
                        target_rot += rot_correct_to_right * dt / auto_robot_line_tracker_correction_time;
                      }
                    }};
    while (true)
    {
      input();
      auto const line_left{line_sensor_left.read()}, line_right{line_sensor_right.read()};
      if (line_left && line_right)
      {
        break;
      }
      track_line(line_left, line_right);
      target_pos += auto_robot_translation_velocity * dt;
      output("moving");
    }

    // Navigation
    target_pos += auto_robot_navigation_initial_translation;
    while (true)
    {
      input();
      if (std::abs(target_pos - move_adrc.m_position) <= auto_robot_translation_tolerance)
      {
        break;
      }
      output("navigating");
    }
    while (!box_targets)
    {
      input();
      output("waiting");
    }
    long cur_line{4}; // 0, line, 2, line, 4, line, 6, line, 8
    auto detect_line_change{[&, last_change{time.time()}, last_line{line_sensor_mid.read()}, last_confirmed_line{line_sensor_mid.read()}](bool line) mutable
                            {
                              if (last_line != line)
                              {
                                last_line = line;
                                last_change = time.time();
                              }
                              if (last_confirmed_line != last_line && time.time() - last_change >= auto_robot_line_sensor_filter_time)
                              {
                                last_confirmed_line = last_line;
                                return true;
                              }
                              return false;
                            }};
    for (auto const target : *box_targets)
    {
      while (cur_line != target)
      {
        input();
        long direction{target - cur_line};
        if (detect_line_change(line_sensor_mid.read()))
        {
          cur_line += std::copysign(1, direction);
        }
        target_rot += std::copysign(auto_robot_navigation_angular_velocity * dt, -direction);
        output("navigating");
      }
      target_pos += auto_robot_navigation_approach_translation;
      while (std::abs(target_pos - move_adrc.m_position) > auto_robot_translation_tolerance)
      {
        input();
        // track_line(line_sensor_left.read(), line_sensor_right.read());
        output("navigating");
      }
      auto const last_time{time.time()};
      while (time.time() - last_time <= 3.)
      {
        input();
        output("throwing");
      }
      target_pos -= auto_robot_navigation_approach_translation;
      while (std::abs(target_pos - move_adrc.m_position) > auto_robot_translation_tolerance)
      {
        input();
        // track_line(line_sensor_left.read(), line_sensor_right.read());
        output("navigating");
      }
    }

    // Completion
    while (true)
    {
      input();
      active = false;
      output("completion");
    }
  }

  namespace
  {
    constexpr auto const task_robot_translation_velocity{1.3};
    constexpr auto const task_robot_rotation_velocity{math::tau / 3};
    constexpr auto const task_robot_valve_reversed{false};
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
    TaskRobotADRC move_adrc{{0., 0.}, math::tau / 4., {motors_r[0].get_velocity(), motors_r[1].get_velocity(), motors_r[2].get_velocity(), motors_r[3].get_velocity()}};
    GPIO stand{VALVE1_GPIO_Port, VALVE1_Pin, task_robot_valve_reversed}, grab1{VALVE2_GPIO_Port, VALVE2_Pin, task_robot_valve_reversed}, grab2{VALVE3_GPIO_Port, VALVE3_Pin, task_robot_valve_reversed};

    auto dt{0.};
    auto active{true};
    auto automode{false};
    auto stand_mode{false};
    auto grab1_mode{false};
    auto grab2_mode{false};
    math::Vector<double, 2> target_pos{};
    double target_rot{math::tau / 4.};

    Commander<3> commander{};
    Receiver<16, false> receiver{huart1, .05};
    // Turn ON/OFF
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
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{0., task_robot_translation_velocity * dt};
                     });
    commander.handle('a',
                     [&wasd_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{wasd_command_capture};
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{-task_robot_translation_velocity * dt, 0.};
                     });
    commander.handle('s',
                     [&wasd_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{wasd_command_capture};
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{0., -task_robot_translation_velocity * dt};
                     });
    commander.handle('d',
                     [&wasd_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{wasd_command_capture};
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{task_robot_translation_velocity * dt, 0.};
                     });

    // rotate anticlockwise
    commander.handle('o',
                     [&dt, &target_rot](typename decltype(commander)::ParamType const &)
                     {
                       target_rot += task_robot_rotation_velocity * dt;
                     });

    // rotote clockwise
    commander.handle('p',
                     [&dt, &target_rot](typename decltype(commander)::ParamType const &)
                     {
                       target_rot += -task_robot_rotation_velocity * dt;
                     });

    auto qezc_command_capture{std::tie(dt, target_pos, target_rot)};

    // move to NW direction
    commander.handle('q',
                     [&qezc_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{qezc_command_capture};
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{-task_robot_translation_velocity * dt, task_robot_translation_velocity * dt};
                     });

    // move to NE direction
    commander.handle('e',
                     [&qezc_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{qezc_command_capture};
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{task_robot_translation_velocity * dt, task_robot_translation_velocity * dt};
                     });

    // move to SW direction
    commander.handle('z',
                     [&qezc_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{qezc_command_capture};
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{-task_robot_translation_velocity * dt, -task_robot_translation_velocity * dt};
                     });

    // move to SE direction
    commander.handle('c',
                     [&qezc_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{qezc_command_capture};
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{task_robot_translation_velocity * dt, -task_robot_translation_velocity * dt};
                     });

    // brake
    commander.handle('b',
                     [&qezc_command_capture](typename decltype(commander)::ParamType const &)
                     {
                       auto &[dt, target_pos, target_rot]{qezc_command_capture};
                       // todo: does not work as intended
                       target_pos += math::rotation_matrix2(target_rot - math::tau / 4.) * std::remove_reference_t<decltype(target_pos)>{0., 0.};
                     });

    // grab 1 seedlings
    commander.handle('k',
                     [&grab1_mode](typename decltype(commander)::ParamType const &)
                     {
                       grab1_mode = !grab1_mode;
                     });

    // grab 2 seedlings
    commander.handle('l',
                     [&grab2_mode](typename decltype(commander)::ParamType const &)
                     {
                       grab2_mode = !grab2_mode;
                     });

    // Controling the stand of holding grabs
    commander.handle('y',
                     [&stand_mode](typename decltype(commander)::ParamType const &)
                     {
                       stand_mode = !stand_mode;
                     });

    // Auto Shortcut
    commander.handle('n',
                     [&automode](typename decltype(commander)::ParamType const &)
                     {
                       automode = !automode;
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

      // auto shortcut
      if (automode)
      {
        // receive the distance of two tof sensors
        // compare two distances
        // Method 1 (receive the distance constantly and keep the robot to face to N direction)
        // front > back (face to NE direction)
        // change the value of target_rot to make the robot face to N direction
        // back > front (face to NW direction)
        // change the value of target_rot to make the robot face to N direction
        // front == back (face to N direction)
        // change the value of target_pos to move in N direction
        // Method 2 (receive the distance constantly and make the robot to move to N direction without rotation)
        // front > back
        // change the value of target_pos to make the robot move in N direction and it doesnt rotate
        // front < back
        // change the value of target_pos to make the robot move in N direction and it doesnt rotate
        // front == back
        // change the value of target_pos to move in N direction
      }

      if (stand_mode)
        stand.write(true);
      else
        stand.write(false);

      if (grab1_mode)
        grab1.write(true);
      else
        grab1.write(false);

      if (grab2_mode)
        grab2.write(true);
      else
        grab2.write(false);

      CANMotorsControl<4> motors{motors_r};
      auto const [v_fl, v_fr, v_rl, v_rr]{(active * move_adrc.update(target_pos, target_rot, {motors[0].get_velocity(), motors[1].get_velocity(), motors[2].get_velocity(), motors[3].get_velocity()}, dt)).transpose()[0]};
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
        tft_prints(0, 4, "v: %.2f, %.2f", motors[0].get_velocity(), motors[1].get_velocity());
        tft_prints(0, 5, "   %.2f, %.2f", motors[2].get_velocity(), motors[3].get_velocity());
        tft_prints(0, 6, "v_t: %.2f, %.2f", v_fl, v_fr);
        tft_prints(0, 7, "     %.2f, %.2f", v_rl, v_rr);
      }
    }
  }
}
