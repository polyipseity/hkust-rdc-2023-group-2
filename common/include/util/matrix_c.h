#pragma once

#include <stdalign.h>
#include <stddef.h>

#include "globals.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MATRIX(Scalar, rows, cols, ...) math_Matrix_##Scalar##_##rows##_##cols##__VA_ARGS__
#define DECLARE_MATRIX(Scalar, rows, cols)                                        \
  typedef struct MATRIX(Scalar, rows, cols) MATRIX(Scalar, rows, cols);           \
  typedef struct MATRIX(Scalar, rows, cols, _storage)                             \
  {                                                                               \
    alignas(alignof(Scalar)) unsigned char storage[sizeof(Scalar) * rows * cols]; \
  } MATRIX(Scalar, rows, cols, _storage);                                         \
                                                                                  \
  __attribute__((warn_unused_result)) MATRIX(Scalar, rows, cols) * MATRIX(Scalar, rows, cols, _new)(MATRIX(Scalar, rows, cols, _storage) * RESTRICT self) NOEXCEPT;

  DECLARE_MATRIX(double, 2, 1);

#undef DECLARE_MATRIX

#ifdef __cplusplus
}
#endif
