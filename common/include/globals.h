#pragma once

#ifdef __cplusplus
#define CONSTEXPR constexpr static
#define NOEXCEPT noexcept
#define RESTRICT __restrict__
#else
#define CONSTEXPR static
#define NOEXCEPT
#define RESTRICT restrict
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Minimum TFT update interval
   */
  CONSTEXPR double const tft_update_period = 10;

#ifdef __cplusplus
}
#endif
