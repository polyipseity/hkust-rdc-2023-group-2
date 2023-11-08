#pragma once

#include <cinttypes>

#include "main.h"

/**
 * @brief A GPIO device
 */
struct GPIO
{
  /**
   * @brief GPIO type
   */
  GPIO_TypeDef *m_type;

  /**
   * @brief GPIO pin
   */
  std::uint16_t m_pin;

  /**
   * @brief Swap set and reset
   */
  bool m_reversed;

  /**
   * @brief Read from GPIO
   *
   * @return true if set
   * @return false if reset
   */
  auto read [[nodiscard]] () const noexcept -> bool;

  /**
   * @brief Write to GPIO
   *
   * @param state set if true or reset if false
   */
  auto write(bool state) noexcept -> void;
};
