#include "util/adrc_c.h"

#include <new>

#include "util/adrc.hpp"
#include "util/matrix_c.h"

extern "C"
{
  static_assert(sizeof(control::ESO2d) == sizeof(control_ESO2d_storage));
  static_assert(alignof(control::ESO2d) == alignof(control_ESO2d_storage));

  control_ESO2d *control_ESO2d_new [[nodiscard]] (control_ESO2d_storage *self, double gain, double convergence, MATRIX(double, 2, 1) const *state) noexcept
  {
    return reinterpret_cast<control_ESO2d *>(new (&self->storage) control::ESO2d{gain, convergence, *std::launder(reinterpret_cast<math::Matrix<double, 2, 1> const *>(state))});
  }

  void control_ESO2d_update(control_ESO2d *self, double input, double output, double dt) noexcept
  {
    return std::launder(reinterpret_cast<control::ESO2d *>(self))->update(input, output, dt);
  }

  static_assert(sizeof(control::ADRC2d) == sizeof(control_ADRC2d_storage));
  static_assert(alignof(control::ADRC2d) == alignof(control_ADRC2d_storage));

  control_ADRC2d *control_ADRC2d_new [[nodiscard]] (control_ADRC2d_storage *self, double gain, double convergence, MATRIX(double, 2, 1) const *state) noexcept
  {
    return reinterpret_cast<control_ADRC2d *>(new (&self->storage) control::ADRC2d{gain, convergence, *std::launder(reinterpret_cast<math::Matrix<double, 2, 1> const *>(state))});
  }

  double control_ADRC2d_control [[nodiscard]] (control_ADRC2d const *self, double target) noexcept
  {
    return std::launder(reinterpret_cast<control::ADRC2d const *>(self))->control(target);
  }

  double control_ADRC2d_update(control_ADRC2d *self, double target, double input, double output, double dt) noexcept
  {
    return std::launder(reinterpret_cast<control::ADRC2d *>(self))->update(target, input, output, dt);
  }
}
