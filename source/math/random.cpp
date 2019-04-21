
#include "random.h"

namespace base {

thread_local uint64 g_random_seed = 521288629;

void set_seed(uint64 seed) { 
#if !DEBUG
    g_random_seed = seed;
#endif
}

uint64 random_integer() {
    // Pseudo random number generator invented by by George Marsaglia.
    // See https://en.wikipedia.org/wiki/Xorshift.
    uint64 x = g_random_seed;
    x ^= x << 12;
    x ^= x >> 25;
    x ^= x << 27;
    g_random_seed = x;
    return x * 0x2545F4914F6CDD1D;
}

float32 random_float() {
  return ((float32)(random_integer() % BASE_MAX_INT32)) / BASE_MAX_INT32;
}

int64 random_integer_range(int32 imin, int32 imax) {
  if (BASE_PARAM_CHECK) {
    if (imax < imin) {
      return 0;
    }
  }

  if (imin == imax) return imin;
  uint32 span = (imax - imin) + 1;
  return ((int32)imin + (random_integer() % span));
}

float32 random_float_range(float32 imin, float32 imax) {
  if (BASE_PARAM_CHECK) {
    if (imax < imin) {
      return 0.0f;
    }
  }

  return imin + random_float() * (imax - imin);
}

}  // namespace base