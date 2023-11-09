#pragma once

#include <inttypes.h>

#include "globals.h"
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Panic
   *
   * @param message message to display
   */
  __attribute__((noreturn)) void panic(char const *RESTRICT message) NOEXCEPT;

  /**
   * @brief Time tracking
   */
  typedef struct Time
  {
    /**
     * @brief Last update tick
     */
    uint32_t m_last_tick;
  } Time;

  /**
   * @brief Get last time
   *
   * @return last time in seconds
   */
  __attribute__((warn_unused_result)) double Time_last_time(Time const *RESTRICT self) NOEXCEPT;

  /**
   * @brief Get current time
   *
   * @return current time in seconds
   */
  __attribute__((warn_unused_result)) double Time_time(Time const *RESTRICT self) NOEXCEPT;

  /**
   * @brief Update last time
   *
   * @return time since last update in seconds
   */
  double Time_update(Time *RESTRICT self) NOEXCEPT;

#ifdef __cplusplus
}
#endif
