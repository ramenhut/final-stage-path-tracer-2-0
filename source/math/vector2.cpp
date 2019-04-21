
#include "vector2.h"
#include "vector3.h"
#include "vector4.h"

namespace base {

vector2::vector2() {
  x = 0;
  y = 0;
}

vector2::vector2(const vector2& rhs) { (*this) = rhs; }

vector2::vector2(float32 xj, float32 yj) {
  x = xj;
  y = yj;
}

vector2::~vector2() {}

}  // namespace base