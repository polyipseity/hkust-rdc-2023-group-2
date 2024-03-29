#include "cpp_main.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <tuple>

#include "communication.hpp"
#include "gpio_ext.hpp"
#include "lcd/lcd.h"
#include "mains.hpp"
#include "motor.hpp"
#include "transform.hpp"
#include "usart.h"
#include "util.hpp"
#include "util/math.hpp"

namespace
{
    constexpr auto const auto_robot_line_sensor_reversed{false};
    constexpr auto const auto_robot_initial_delay{2.};
    constexpr auto const auto_robot_translation_velocity{.2};
    constexpr auto const auto_robot_translation_tolerance{.01};
    constexpr auto const auto_robot_rotation_velocity{math::tau / 16.};
    constexpr auto const auto_robot_rotation_tolerance{math::tau / 64.};

    constexpr auto const auto_robot_initial_translation{.2};
    constexpr auto const auto_robot_line_tracker_correction{math::tau / 64.};
    constexpr auto const auto_robot_line_tracker_correction_time{.1};
    constexpr auto const auto_robot_line_tracker_delay_time{.1};
    constexpr auto const auto_robot_avg_forward_rotation_time{4.};

    constexpr auto const auto_robot_navigation_initial_translation{.3};
    constexpr auto const auto_robot_navigation_side_translation{.1};
    constexpr auto const auto_robot_navigation_angular_velocity{math::tau / 32.};
    constexpr auto const auto_robot_line_sensor_filter_time{.15};
    constexpr auto const auto_robot_navigation_rotation_inner_offset{math::tau / 32.};

    constexpr auto const auto_robot_thrower_velocity{math::tau}; // For safety, do not remove.
    constexpr std::array<double, 2> const auto_robot_thrower_offsets{math::tau / 6., math::tau / 2. + math::tau / 64.};
    constexpr auto const auto_robot_thrower_confirmation_time{1.};

    /**
     * @brief Code for auto robot
     */
    auto auto_robot [[noreturn]] () noexcept
    {
        CANMotors<3> motors_r{{CAN1_MOTOR1, CAN1_MOTOR0, CAN2_MOTOR2},
                              {false, true}};
        std::array<control::ADRC2d, 3> motor_adrcs{
            new_motor_ADRC_auto(motors_r[0]),
            new_motor_ADRC_auto(motors_r[1]),
            new_motor_ADRC_auto(motors_r[2], 4., 70.),
        };
        AutoRobotADRC move_adrc{0., math::tau / 4., {motors_r[0].get_velocity(), motors_r[1].get_velocity()}};
        PositionADRC thrower_adrc{0., motors_r[2].get_velocity(), math::tau / 31., 2.};
        GPIO button1{BTN1_GPIO_Port, BTN1_Pin, true},
            button2{BTN2_GPIO_Port, BTN2_Pin, true},
            line_sensor_left{CAM_D1_GPIO_Port, CAM_D1_Pin, auto_robot_line_sensor_reversed},
            line_sensor_right{CAM_D3_GPIO_Port, CAM_D3_Pin, auto_robot_line_sensor_reversed},
            line_sensor_mid{CAM_D5_GPIO_Port, CAM_D5_Pin, auto_robot_line_sensor_reversed};

        auto dt{0.};
        auto active{true};
        double target_pos{}, target_rot{math::tau / 4.}, thrower_rot{};
        std::optional<std::array<long, 2>> box_targets{};

        Commander<2> commander{};
        Receiver<16, false> receiver{huart1, .05};
        commander.handle('x',
                         [&active, &receiver](typename decltype(commander)::ParamType const &) {
                             receiver.invalidate();
                             active = !active;
                         });
        commander.handle('t',
                         [&box_targets, &receiver](typename decltype(commander)::ParamType const &params) {
                             receiver.invalidate();
                             if (box_targets) {
                                 return;
                             }
                             auto &&[param0, param1]{params};
                             char *end0{}, *end1{};
                             auto const tar0{std::strtol(std::get<1>(param0), &end0, 10)},
                                 tar1{std::strtol(std::get<1>(param1), &end1, 10)};
                             if (std::get<1>(param0) == end0 || std::get<1>(param1) == end1) {
                                 return;
                             }
                             box_targets = {tar0, tar1}; // 1, 2, 3, 4
                         });

        Time time{};
        auto const input{[&]() {
            dt = time.update();
            auto const [received_size, received]{receiver.update()};
            commander.dispatch(received, received_size);
        }};
        auto const output{[&](char const state[]) {
            if (!active) {
                target_pos = move_adrc.m_position;
                target_rot = math::rotation_matrix2_angle(move_adrc.m_rotation);
            }
            CANMotorsControl<3> motors{motors_r};
            auto const [v_l, v_r]{move_adrc.update(target_pos, target_rot, {motors[0].get_velocity(), motors[1].get_velocity()}, dt, {auto_robot_translation_velocity, auto_robot_rotation_velocity})};
            update_motor_velocity(motors[0], motor_adrcs[0], active * v_l, dt);
            update_motor_velocity(motors[1], motor_adrcs[1], active * v_r, dt);
            auto const thrower_v{thrower_adrc.update(thrower_rot, motors[2].get_velocity(), dt)};
            update_motor_velocity(motors[2], motor_adrcs[2], active * std::copysign(std::min(auto_robot_thrower_velocity / thrower_adrc.m_gain, std::abs(thrower_v)), thrower_v), dt);

            if (tft_update(tft_update_period)) {
                tft_prints(0, 0, "pos: %.2f", move_adrc.m_position);
                tft_prints(0, 1, "pos_t: %.2f", target_pos);
                tft_prints(0, 2, "rot: %.2f", math::rotation_matrix2_angle(move_adrc.m_rotation));
                tft_prints(0, 3, "rot_t: %.2f", target_rot);
                tft_prints(0, 4, "v: %.2f, %.2f", motors[0].get_velocity(), motors[1].get_velocity());
                tft_prints(0, 5, "v_t: %.2f, %.2f", v_l, v_r);
                tft_prints(0, 6, "sensor: %d, %d, %d", line_sensor_left.read(), line_sensor_mid.read(), line_sensor_right.read());
                tft_prints(0, 7, "targets: %d, %d", box_targets ? static_cast<int>((*box_targets)[0]) : -1, box_targets ? static_cast<int>((*box_targets)[1]) : -1);
                tft_prints(0, 8, "%s", state);
            }
        }};

        // Testing
        if (button1.read()) {
            auto auto_robot_thrower_offset_iter{std::cbegin(auto_robot_thrower_offsets)};
            while (true) {
                if (auto_robot_thrower_offset_iter == std::cend(auto_robot_thrower_offsets)) {
                    while (true) {
                        input();
                        output("testing");
                    }
                } else {
                    thrower_rot += *auto_robot_thrower_offset_iter++;
                }
                auto last_match{-auto_robot_thrower_confirmation_time};
                while (true) {
                    input();
                    if (std::abs(thrower_rot - thrower_adrc.m_position) <= auto_robot_rotation_tolerance * 2.) {
                        if (last_match <= 0.) {
                            last_match = time.time();
                        }
                        if (last_match >= 0. && time.time() - last_match >= auto_robot_thrower_confirmation_time) {
                            break;
                        }
                    } else {
                        last_match = -auto_robot_thrower_confirmation_time;
                    }
                    output("throwing");
                }
            }
        }

        // Initialization
        auto const stationary_mode{button2.read()};
        auto const initial_time{time.time()};
        while (time.time() - initial_time <= auto_robot_initial_delay) {
            input();
            output("initializing");
        }

        if (!stationary_mode) {
            // Movement
            target_pos += auto_robot_initial_translation;
            while (true) {
                input();
                if (std::abs(target_pos - move_adrc.m_position) <= auto_robot_translation_tolerance) {
                    break;
                }
                output("moving");
            }
        }
        auto track_line{[&, last_line_left{line_sensor_left.read()}, last_line_right{line_sensor_right.read()}, last_change{time.time()}](bool line_left, bool line_right) mutable {
            if (last_line_left != line_left) {
                last_line_left = line_left;
                last_change    = time.time();
            }
            if (last_line_right != line_right) {
                last_line_right = line_right;
                last_change     = time.time();
            }
            if (time.time() - last_change <= auto_robot_line_tracker_delay_time) {
                return false;
            }
            if (line_left) {
                target_rot += auto_robot_line_tracker_correction * dt / auto_robot_line_tracker_correction_time;
            }
            if (line_right) {
                target_rot -= auto_robot_line_tracker_correction * dt / auto_robot_line_tracker_correction_time;
            }
            return line_left && line_right;
        }};
        if (!stationary_mode) {
            auto avg_forward_rotation{math::rotation_matrix2_angle(move_adrc.m_rotation)};
            while (true) {
                input();
                auto const line_left{line_sensor_left.read()}, line_right{line_sensor_right.read()};
                if (track_line(line_left, line_right)) {
                    target_rot = avg_forward_rotation;
                    break;
                }
                avg_forward_rotation += (math::rotation_matrix2_angle(move_adrc.m_rotation) - avg_forward_rotation) * dt / auto_robot_avg_forward_rotation_time;
                target_pos += auto_robot_translation_velocity * dt;
                output("moving");
            }

            // Navigation
            target_pos += auto_robot_navigation_initial_translation;
            while (true) {
                input();
                if (std::abs(target_pos - move_adrc.m_position) <= auto_robot_translation_tolerance) {
                    break;
                }
                output("navigating");
            }
        }

        while (!box_targets) {
            input();
            output("waiting");
        }
        long cur_line{4}; // 0, line, 2, line, 4, line, 6, line, 8
        if (line_sensor_mid.read()) {
            bool modified{};
            if (line_sensor_left.read()) {
                --cur_line;
                modified = true;
            }
            if (line_sensor_right.read()) {
                ++cur_line;
                modified = true;
            }
            if (!modified) {
                ++cur_line; // assume left bias
            }
        }
        auto detect_line_change{[&, last_change{time.time()}, last_line{line_sensor_mid.read()}, last_confirmed_line{line_sensor_mid.read()}](bool line) mutable {
            if (last_line != line) {
                last_line   = line;
                last_change = time.time();
            }
            if (last_confirmed_line != last_line && time.time() - last_change >= auto_robot_line_sensor_filter_time) {
                last_confirmed_line = last_line;
                return true;
            }
            return false;
        }};
        auto auto_robot_thrower_offset_iter{std::cbegin(auto_robot_thrower_offsets)};
        for (auto const target : *box_targets) {
            while (cur_line != target) {
                input();
                long direction{target - cur_line};
                if (detect_line_change(line_sensor_mid.read())) {
                    cur_line += std::copysign(1, direction);
                }
                target_rot += std::copysign(auto_robot_navigation_angular_velocity * dt, -direction);
                output("navigating");
            }
            auto const inner_direction{target == 1. ? -1. : target == 7 ? 1
                                                        : target == 3   ? -.5
                                                        : target == 5   ? .5
                                                                        : 0.};
            target_rot += inner_direction * auto_robot_navigation_rotation_inner_offset;
            while (true) {
                input();
                if (std::fmod(std::abs(math::rotation_matrix2_angle(move_adrc.m_rotation) - target_rot), math::tau) <= auto_robot_rotation_tolerance) {
                    break;
                }
                output("navigating");
            }
            if (target == 1 || target == 7) {
                target_pos += auto_robot_navigation_side_translation;
            } else if (target == 3 || target == 5) {
                target_pos += auto_robot_navigation_side_translation / 1.4;
            }
            while (std::abs(target_pos - move_adrc.m_position) > auto_robot_translation_tolerance) {
                input();
                track_line(line_sensor_left.read(), line_sensor_right.read());
                output("navigating");
            }
            thrower_rot += *auto_robot_thrower_offset_iter++;
            auto last_match{-auto_robot_thrower_confirmation_time};
            while (true) {
                input();
                if (std::abs(thrower_rot - thrower_adrc.m_position) <= auto_robot_rotation_tolerance * 2.) {
                    if (last_match <= 0.) {
                        last_match = time.time();
                    }
                    if (last_match >= 0. && time.time() - last_match >= auto_robot_thrower_confirmation_time) {
                        break;
                    }
                } else {
                    last_match = -auto_robot_thrower_confirmation_time;
                }
                output("throwing");
            }
            if (target == 1 || target == 7) {
                target_pos -= auto_robot_navigation_side_translation;
            } else if (target == 3 || target == 5) {
                target_pos -= auto_robot_navigation_side_translation / 1.4;
            }
            while (std::abs(target_pos - move_adrc.m_position) > auto_robot_translation_tolerance) {
                input();
                output("navigating");
            }
            if (inner_direction && !line_sensor_mid.read()) {
                while (!detect_line_change(false))
                    ;
                cur_line += -inner_direction;
            }
        }

        // Completion
        while (true) {
            input();
            active = false;
            output("completion");
        }
    }

    /**
     * @brief The implementation for `cpp_main`
     *
     * @return exit code
     */
    auto cpp_main2 [[nodiscard]] () noexcept
    {
        auto_robot();
        return 0;
    }
}

extern "C" {
auto cpp_main [[nodiscard]] () noexcept -> int
{
    return cpp_main2();
}
}
