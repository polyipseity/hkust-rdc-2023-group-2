#pragma once

#include <inttypes.h>
#include <stdbool.h>

#include "globals.h"
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief A GPIO device
   */
  typedef struct GPIO
  {
    /**
     * @brief GPIO type
     */
    GPIO_TypeDef *m_type;

    /**
     * @brief GPIO pin
     */
    uint16_t m_pin;

    /**
     * @brief Swap set and reset
     */
    bool m_reversed;
  } GPIO;

  /**
   * @brief Read from GPIO
   *
   * @param self `GPIO`
   * @return true if set
   * @return false if reset
   */
  __attribute__((warn_unused_result)) bool GPIO_read(GPIO const *RESTRICT self) NOEXCEPT;

  /**
   * @brief Write to GPIO
   *
   * @param self `GPIO`
   * @param state set if true or reset if false
   */
  void GPIO_write(GPIO *RESTRICT self, bool state) NOEXCEPT;

#ifdef __cplusplus
}
#endif
