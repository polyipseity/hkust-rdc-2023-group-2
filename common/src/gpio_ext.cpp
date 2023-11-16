#include "gpio_ext.hpp"

#include "main.h"

auto GPIO::read [[nodiscard]] () const noexcept -> bool
{
  return HAL_GPIO_ReadPin(m_type, m_pin) == (m_reversed ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

auto GPIO::write(bool state) noexcept -> void
{
  HAL_GPIO_WritePin(m_type, m_pin, state == m_reversed ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

auto GPIO::toggle() noexcept -> void
{
  HAL_GPIO_TogglePin(m_type, m_pin);
}
