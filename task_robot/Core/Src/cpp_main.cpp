#include "cpp_main.h"

#include <array>
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
    constexpr auto const task_robot_translation_velocity{1.3};
    constexpr auto const task_robot_rotation_velocity{math::tau / 2.};
    constexpr auto const task_robot_valve_reversed{false};

    /**
     * @brief Code for task robot
     */
    auto task_robot [[noreturn]] () noexcept
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

        Commander<8> commander{};
        Receiver<27, false> receiver{huart1, .025};

        std::array<bool, 4> ctrl_btns{};
        auto controller_capture{std::tie(active, automode, ctrl_btns, dt, move_adrc, target_pos, target_rot)};
        commander.handle('c',
                         [&controller_capture](typename decltype(commander)::ParamType const &params) {
                             auto &[active, automode, ctrl_btns, dt, move_adrc, target_pos, target_rot]{controller_capture};
                             /*
                             controller
                             - left joystick: movement
                             - L2: rotate left
                             - R2: rotate right
                             - X button: toggle the active status
                             - triangle button: toggle auto shortcut mode

                             message
                             c(num0),(num1),(num2),(num3),(num4),(num5),(num6),(num7)\n
                             - length: 1 + (4 + 1) * 2 + (3 + 1) * 2 + (1 + 1) * 4 - 1 + 1
                             - num0: left joystick X (right is positive), [-100, 100]
                             - num1: left joystick Y (down is positive), [-100, 100]
                             - num2: R2, [0, 100]
                             - num3: L2, [0, 100]
                             - num4: X button, [0, 1]
                             - num5: circle button, [0, 1]
                             - num6: square button, [0, 1]
                             - num7: triangle button, [0, 1]
                             */
                             auto &&[num0, num1, num2, num3, num4, num5, num6, num7]{params};
                             std::array<char *, 8> ends{};
                             long l_js_x{std::strtol(std::get<1>(num0), &ends[0], 10)},
                                 l_js_y{std::strtol(std::get<1>(num1), &ends[1], 10)},
                                 r2{std::strtol(std::get<1>(num2), &ends[2], 10)},
                                 l2{std::strtol(std::get<1>(num3), &ends[3], 10)},
                                 x_btn{std::strtol(std::get<1>(num4), &ends[4], 10)},
                                 circle_btn{std::strtol(std::get<1>(num5), &ends[5], 10)},
                                 square_btn{std::strtol(std::get<1>(num6), &ends[6], 10)},
                                 triangle_btn{std::strtol(std::get<1>(num7), &ends[7], 10)};
                             for (std::size_t ii{}; ii < 8; ++ii) {
                                 if (std::get<1>(params[ii]) == ends[ii]) {
                                     return;
                                 }
                             }
                             // movement
                             target_pos += move_adrc.m_rotation * math::rotation_matrix2(-math::tau / 4.) *
                                           std::remove_reference_t<decltype(target_pos)>{
                                               l_js_x / 100. * task_robot_translation_velocity * dt,  // x
                                               -l_js_y / 100. * task_robot_translation_velocity * dt, // y
                                           };
                             // rotate anticlockwise
                             target_rot += l2 / 100. * task_robot_rotation_velocity * dt;
                             // rotate clockwise
                             target_rot -= r2 / 100. * task_robot_rotation_velocity * dt;
                             if (!ctrl_btns[0] && x_btn) {
                                 // on/off
                                 active = !active;
                             }
                             if (!ctrl_btns[3] && triangle_btn) {
                                 // auto shortcut mode
                                 automode = !automode;
                             }
                             ctrl_btns = {x_btn != 0, circle_btn != 0, square_btn != 0, triangle_btn != 0};
                         });

        // grab 1 seedlings
        commander.handle('k',
                         [&grab1_mode](typename decltype(commander)::ParamType const &) {
                             grab1_mode = !grab1_mode;
                         });

        // grab 2 seedlings
        commander.handle('l',
                         [&grab2_mode](typename decltype(commander)::ParamType const &) {
                             grab2_mode = !grab2_mode;
                         });

        // Controling the stand of holding grabs
        commander.handle('y',
                         [&stand_mode](typename decltype(commander)::ParamType const &) {
                             stand_mode = !stand_mode;
                         });

        Time time{};
        while (true) {
            dt = time.update();

            auto const [received_size, received]{receiver.update()};
            commander.dispatch(received, received_size);

            if (!active) {
                target_pos = move_adrc.m_position;
                target_rot = math::rotation_matrix2_angle(move_adrc.m_rotation);
            }

            // auto shortcut
            if (automode) {
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

            if (tft_update(tft_update_period)) {
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

    /**
     * @brief The implementation for `cpp_main`
     *
     * @return exit code
     */
    auto cpp_main2 [[nodiscard]] () noexcept
    {
        task_robot();
        return 0;
    }
}

extern "C" {
auto cpp_main [[nodiscard]] () noexcept -> int
{
    return cpp_main2();
}
}
