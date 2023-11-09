#include "gpio_ext.h"

#include <stdbool.h>

#include "main.h"

__attribute__((warn_unused_result)) bool GPIO_read(GPIO const *restrict self)
{
  return HAL_GPIO_ReadPin(self->m_type, self->m_pin) == (self->m_reversed ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void GPIO_write(GPIO *restrict self, bool state)
{
  HAL_GPIO_WritePin(self->m_type, self->m_pin, state == self->m_reversed ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
