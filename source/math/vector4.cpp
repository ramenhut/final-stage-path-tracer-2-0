
#include "vector4.h"

namespace base {

vector4::vector4() {
  x = 0;
  y = 0;
  z = 0;
  w = 0;
}

vector4::vector4(const vector2& rhs) { (*this) = rhs; }

vector4::vector4(const vector4& rhs) { (*this) = rhs; }

vector4::vector4(const vector3& rhs) { (*this) = rhs; }

vector4::vector4(float32 xj, float32 yj, float32 zj, float32 wj) {
  x = xj;
  y = yj;
  z = zj;
  w = wj;
}

vector4::~vector4() {}

vector4::operator vector2() { return vector2(x, y); }

vector4::operator vector3() { return vector3(x, y, z); }

const vector4& vector4::operator=(const vector2& rhs) {
  x = rhs.x;
  y = rhs.y;
  z = 0.0f;
  w = 0.0f;

  return *this;
}

const vector4& vector4::operator=(const vector3& rhs) {
  x = rhs.x;
  y = rhs.y;
  z = rhs.z;
  w = 1.0f;

  return (*this);
}

}  // namespace base