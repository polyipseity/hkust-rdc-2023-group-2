#pragma once

#include <stdalign.h>
#include <stddef.h>

#include "globals.h"
#include "matrix_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief 2nd order extended state observer using `double`
   */
  typedef struct control_ESO2d control_ESO2d;

  typedef struct control_ESO2d_storage
  {
    alignas(max_align_t) unsigned char storage[80];
  } control_ESO2d_storage;

  /**
   * @brief Construct an extended state observer
   *
   * @param self `control_ESO2d_storage`
   * @param gain rate of change in the (order - 1)-th state (velocity for 2nd order, acceleration for 3rd order) per a unit of input, i.e. the rate of change in the (order - 1)-th state divided by the input
   * @param convergence convergence rate of the estimated state to the actual state
   * @param State current state
   * @return `control_ESO2d`
   */
  __attribute__((warn_unused_result)) control_ESO2d *control_ESO2d_new(control_ESO2d_storage *RESTRICT self, double gain, double convergence, MATRIX(double, 2, 1) const *RESTRICT state) NOEXCEPT;

  /**
   * @brief Update the estimated state
   *
   * @param self `control_ESO2d`
   * @param input current input
   * @param output current output
   * @param dt time since last update
   */
  void control_ESO2d_update(control_ESO2d *RESTRICT self, double input, double output, double dt) NOEXCEPT;

  /**
   * @brief 2nd order ADRC using `double`
   */
  typedef struct control_ADRC2d control_ADRC2d;

  typedef struct control_ADRC2d_storage
  {
    alignas(max_align_t) unsigned char storage[104];
  } control_ADRC2d_storage;

  /**
   * @brief Construct an ADRC controller
   *
   * @param self `control_ADRC2d_storage`
   * @param gain rate of change in the (order - 1)-th state (velocity for 2nd order, acceleration for 3rd order) per a unit of input, i.e. the rate of change in the (order - 1)-th state divided by the input
   * @param convergence convergence rate of the state to the desired state
   * @param state current state
   * @return `control_ADRC2d`
   */
  __attribute__((warn_unused_result)) control_ADRC2d *control_ADRC2d_new(control_ADRC2d_storage *RESTRICT self, double gain, double convergence, MATRIX(double, 2, 1) const *RESTRICT state) NOEXCEPT;

  /**
   * @brief Recommend an input to reach the desired state
   *
   * @param self `control_ADRC2d`
   * @param target desired state
   * @return a recommended input
   */
  __attribute__((warn_unused_result)) double control_ADRC2d_control(control_ADRC2d const *RESTRICT self, double target) NOEXCEPT;

  /**
   * @brief Update the extended state observer and recommend an input to reach the desired state
   *
   * @param self `control_ADRC2d`
   * @param target desired state
   * @param input current input
   * @param output current output
   * @param dt time since last update
   * @return a recommended input
   */
  double control_ADRC2d_update(control_ADRC2d *RESTRICT self, double target, double input, double output, double dt) NOEXCEPT;

#ifdef __cplusplus
}
#endif
