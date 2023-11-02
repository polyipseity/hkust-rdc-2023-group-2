#include "cpp_main.h"

#include <array>

#include "can.h"
#include "lcd/lcd.h"
#include "main.h"
#include "motor.hpp"
#include "position.hpp"
#include "receiver.hpp"
#include "util/adrc.hpp"
#include "usart.h"

namespace
{
    constexpr std::size_t const motor_size{1};
    constexpr auto const tft_update_period{10};

    auto find_motor_gain [[maybe_unused]] [[nodiscard]] (Motor motor_handle) noexcept
    {
        constexpr static auto const input{.5}, duration{1.};
        CANMotors<1> motors{{motor_handle}};
        auto const start_tick{HAL_GetTick()};
        while ((HAL_GetTick() - start_tick) / 1000. <= duration) {
            CANMotorsControl<1> motors_ctrl{motors};
            auto &motor{motors_ctrl[0]};
            motor.setInput(input);
        }
        auto velocity{0.};
        while (velocity == 0.) {
            CANMotorsControl<1> motors_ctrl{motors};
            auto &motor{motors_ctrl[0]};
            motor.setInput(0);
            velocity = motor.getVelocity();
        }
        while (true) {
            if (tft_update(tft_update_period)) {
                tft_prints(0, 0, "input: %.6f", input);
                tft_prints(0, 1, "duration: %.6f", duration);
                tft_prints(0, 2, "velocity: %.6f", velocity);
                tft_prints(0, 3, "gain: %.6f", velocity / input / duration / 2.);
            }
        }
    }

    auto cpp_main2 [[nodiscard]] () noexcept
    {
        CANMotors<motor_size> motors{{CAN1_MOTOR0}};
        std::array<control::ADRC2d, 1> motor_adrcs{
            new_motor_ADRC_auto(motors[0]),
        };
        PositionADRC pos_adrc{0., 0.};
        Receiver receiver{huart1};

        auto last_tick{HAL_GetTick()};
        auto target_position{0.};
        while (true) {
            CANMotorsControl<motor_size> motors_ctrl{motors};
            auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
            auto const dt{elapsed / 1000.};
            receiver.update();

            if (!btn_read(BTN1)) {
                target_position -= 100. * dt;
            }
            if (!btn_read(BTN2)) {
                target_position += 100. * dt;
            }

            for (std::size_t ii{}; ii < motor_size; ++ii) {
                auto &motor{motors_ctrl[ii]};
                update_motor_velocity(motor, motor_adrcs[ii], pos_adrc.update(target_position, motor.getVelocity(), dt), dt);
            }
            if (tft_update(tft_update_period)) {
                tft_prints(0, 0, "tick: %u", static_cast<unsigned int>(HAL_GetTick()));
                tft_prints(0, 1, "input: %.3f", motors[0].getInput());
                tft_prints(0, 2, "vel: %.3f", motors[0].getVelocity());
                tft_prints(0, 3, "pos: %.3f", pos_adrc.m_position);
                tft_prints(0, 4, "t pos: %.3f", target_position);
                tft_prints(0, 5, "diff: %.3f", pos_adrc.m_position - target_position);
            }

            last_tick = tick;
            HAL_Delay(1);
        }

        return 0;
    }
}

extern "C" {
auto cpp_main [[nodiscard]] () noexcept -> int
{
    return cpp_main2();
}
}
