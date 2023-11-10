#include "util.hpp"

namespace std
{
  auto __throw_bad_function_call()
  {
    panic("__throw_bad_function_call");
  }
}
