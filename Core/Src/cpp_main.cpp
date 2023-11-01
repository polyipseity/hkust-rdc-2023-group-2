#include "cpp_main.h"

#include <algorithm>
#include <array>

#include "can.h"
#include "lcd/lcd.h"
#include "main.h"
#include "motor.hpp"
#include "position.hpp"
#include "util/adrc.hpp"

namespace
{
    constexpr std::size_t const motor_size{1};

    int cpp_main2()
    {
        CANMotors<motor_size> motors{{CAN1_MOTOR1}};
        std::array<control::ADRC2f, 1> motor_adrcs{
            new_motor_ADRC(motors[0]),
        };
        PositionADRC pos_adrc{0., 0.};

        auto last_tick{HAL_GetTick()};
        auto target_position{0.};
        while (true) {
            CANMotorsControl<motor_size> motors_ctrl{motors};
            auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
            auto const dt{elapsed / 1000.};

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
            if (tft_update(10)) {
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
int cpp_main()
{
    return cpp_main2();
}
}
