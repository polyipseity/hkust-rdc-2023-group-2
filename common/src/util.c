#include "util.h"

#include <inttypes.h>

#include "globals.h"
#include "lcd/lcd.h"
#include "main.h"

__attribute__((noreturn)) void panic(char const *restrict message)
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

__attribute__((warn_unused_result)) double Time_last_time(Time const *restrict self)
{
  return self->m_last_tick / 1000.;
}

__attribute__((warn_unused_result)) double Time_time(Time const *restrict self)
{
  return HAL_GetTick() / 1000.;
}

double Time_update(Time *restrict self)
{
  if (self->m_last_tick == 0)
  {
    self->m_last_tick = HAL_GetTick();
  }
  uint32_t tick = HAL_GetTick();
  while (self->m_last_tick == tick)
  {
    tick = HAL_GetTick();
  }
  double const ret = (tick - self->m_last_tick) / 1000.;
  self->m_last_tick = tick;
  return ret;
}
