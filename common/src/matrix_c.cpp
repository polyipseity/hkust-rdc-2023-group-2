#include "util/matrix_c.h"

#include <new>

#include "util/matrix.hpp"

extern "C"
{
#define DEFINE_MATRIX(Scalar, rows, cols)                                                                                            \
  static_assert(sizeof(math::Matrix<Scalar, rows, cols>) == sizeof(MATRIX(Scalar, rows, cols, _storage)));                           \
  static_assert(alignof(math::Matrix<Scalar, rows, cols>) == alignof(MATRIX(Scalar, rows, cols, _storage)));                         \
                                                                                                                                     \
  MATRIX(Scalar, rows, cols) * MATRIX(Scalar, rows, cols, _new) [[nodiscard]] (MATRIX(Scalar, rows, cols, _storage) * self) noexcept \
  {                                                                                                                                  \
    return reinterpret_cast<MATRIX(Scalar, rows, cols) *>(new (&self->storage) math::Matrix<Scalar, rows, cols>{});                  \
  }

  DEFINE_MATRIX(double, 2, 1);

#undef DEFINE_MATRIX
}
