#include "cpp_main.h"

#include "mains.hpp"
#include "usart.h"

namespace
{
    /**
     * @brief The implementation for `cpp_main`
     *
     * @return exit code
     */
    auto cpp_main2 [[nodiscard]] () noexcept
    {
        test::test_motor_velocity(RobotType::AUTO, CAN1_MOTOR0, 4);
        return 0;
    }
}

extern "C" {
auto cpp_main [[nodiscard]] () noexcept -> int
{
    return cpp_main2();
}
}
