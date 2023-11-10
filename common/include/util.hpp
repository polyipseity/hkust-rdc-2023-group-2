#pragma once

#include <type_traits>

#include "main.h"

/**
 * @brief Panic
 *
 * @param message message to display
 */
auto panic [[noreturn]] (char const message[] = "") -> void;

/**
 * @brief Time tracking
 */
class Time
{
  /**
   * @brief Last update tick
   */
  decltype(HAL_GetTick()) m_last_tick{HAL_GetTick()};

public:
  /**
   * @brief Get last time
   *
   * @return last time in seconds
   */
  auto last_time [[nodiscard]] () const noexcept -> double;

  /**
   * @brief Get current time
   *
   * @return current time in seconds
   */
  auto time [[nodiscard]] () const noexcept -> double;

  /**
   * @brief Update last time
   *
   * @return time since last update in seconds
   */
  auto update() noexcept -> double;
};
