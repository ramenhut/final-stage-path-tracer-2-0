
#include "vector3.h"
#include "vector4.h"

namespace base {

vector3::vector3() {
  x = 0;
  y = 0;
  z = 0;
}

vector3::vector3(const vector2& rhs) { (*this) = rhs; }

vector3::vector3(const vector3& rhs) { (*this) = rhs; }

vector3::vector3(float32 xj, float32 yj, float32 zj) {
  x = xj;
  y = yj;
  z = zj;
}

vector3::~vector3() {}

vector3::operator vector2() { return vector2(x, y); }

const vector3& vector3::operator=(const vector2& rhs) {
  x = rhs.x;
  y = rhs.y;
  z = 0.0f;

  return *this;
}

}  // namespace base
