#include "util.h"

namespace std
{
  auto __throw_bad_function_call [[noreturn]] ()
  {
    panic("__throw_bad_function_call");
  }
}
