#include "util.hpp"

#include "lcd/lcd.h"
#include "main.h"
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

auto Time::last_time [[nodiscard]] () const noexcept -> double
{
  return m_last_tick / 1000.;
}

auto Time::time [[nodiscard]] () const noexcept -> double
{
  return HAL_GetTick() / 1000.;
}

auto Time::update() noexcept -> double
{
  auto tick{HAL_GetTick()};
  while (m_last_tick == tick)
  {
    tick = HAL_GetTick();
  }
  m_last_tick = tick;
  return (tick - m_last_tick) / 1000.;
}
