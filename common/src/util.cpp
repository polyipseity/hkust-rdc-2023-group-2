#include "util.hpp"

#include "lcd/lcd.h"
#include "mains.hpp"

auto panic [[noreturn]] (char const message[]) -> void
{
  while (true)
  {
    if (tft_update(tft_update_period))
    {
      tft_prints(0, 0, "PANIC");
      tft_prints(0, 1, "%s", message);
    }
  }
}
