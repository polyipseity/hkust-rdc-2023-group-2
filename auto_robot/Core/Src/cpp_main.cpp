#include "cpp_main.h"

#include "mains.hpp"
#include "usart.h"

namespace
{
    auto cpp_main2 [[nodiscard]] () noexcept
    {
        test::test_uart(huart1);
        return 0;
    }
}

extern "C" {
auto cpp_main [[nodiscard]] () noexcept -> int
{
    return cpp_main2();
}
}
