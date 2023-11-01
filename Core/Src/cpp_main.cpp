#include "cpp_main.h"

#include <array>

#include "can.h"
#include "lcd/lcd.h"
#include "main.h"
#include "motor.hpp"
#include "util/adrc.hpp"

namespace
{
    constexpr std::size_t const motor_size{1};

    int cpp_main2()
    {
        CANMotors<motor_size> motors{{CAN1_MOTOR1}};
        std::array<control::ADRC2f, 1> motor_adrcs{
            new_motor_ADRC(motors[0], 16.),
        };

        auto last_tick{HAL_GetTick()};
        while (true) {
            CANMotorsControl<motor_size> motors_ctrl{motors};
            auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
            auto const dt{elapsed / 1000.};

            for (std::size_t ii{}; ii < motor_size; ++ii) {
                update_motor_velocity(dt, motors_ctrl[ii], motor_adrcs[ii], btn_read(BTN1) ? 0 : 9);
            }
            if (tft_update(10)) {
                tft_prints(0, 0, "tick: %u", static_cast<unsigned int>(HAL_GetTick()));
                tft_prints(0, 1, "input: %.3f", motors[0].getInput());
                tft_prints(0, 2, "pos: %.3f", motors[0].getPosition());
                tft_prints(0, 3, "vel: %.3f", motors[0].getVelocity());
            }

            last_tick = tick;
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
