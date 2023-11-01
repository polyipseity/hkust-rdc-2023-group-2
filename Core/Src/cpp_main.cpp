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
            control::ADRC2f{8., 16., {motors[0].getVelocity()}},
        };
        auto last_tick{HAL_GetTick()};
        while (true) {
            auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
            CANMotorsControl<motor_size> motors_ctrl{motors};
            for (std::size_t ii{}; ii < motor_size; ++ii) {
                auto &motor_ctrl{motors_ctrl[ii]};
                auto const input{motor_adrcs[ii].update(btn_read(BTN1) ? 0. : 4., motor_ctrl.getInput(), motor_ctrl.getVelocity(), elapsed / 1000.)};
                motor_ctrl.setInput(input);
                if (tft_update(10)) {
                    tft_prints(0, 0, "tick: %u", static_cast<unsigned int>(HAL_GetTick()));
                    tft_prints(0, 1, "input: %.3f", motor_ctrl.getInput());
                    tft_prints(0, 2, "pos: %.3f", motor_ctrl.getPosition());
                    tft_prints(0, 3, "vel: %.3f", motor_ctrl.getVelocity());
                }
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
