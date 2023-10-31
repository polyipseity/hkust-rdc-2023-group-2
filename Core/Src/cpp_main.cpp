#include "cpp_main.h"

#include <array>

#include "can.h"
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
            control::ADRC2f{1., 1., {static_cast<float>(motors[0].getPosition())}},
        };
        auto last_tick{HAL_GetTick()};
        while (true) {
            auto const tick{HAL_GetTick()}, elapsed{tick - last_tick};
            CANMotorsControl<motor_size> motors_ctrl{motors};
            for (std::size_t ii{}; ii < motor_size; ++ii) {
                auto &motor_ctrl{motors_ctrl[ii]};
                motor_ctrl.setInput(motor_adrcs[ii].update(10., motor_ctrl.getInput(), motor_ctrl.getPosition(), elapsed / 1000.));
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
