
#include "quaternion.h"

namespace base {

quat::quat() { clear(); }

quat::quat(const quat& rhs) { (*this) = rhs; }

quat::quat(float32 wj, float32 xj, float32 yj, float32 zj) {
  w = wj;
  x = xj;
  y = yj;
  z = zj;
}

quat::quat(const vector3& rhs) {
  x = rhs.x;
  y = rhs.y;
  z = rhs.z;
  w = 0.0f;
}

quat::~quat() {}

matrix3 quat::to_matrix() const {
  float32 x2 = x * x;
  float32 y2 = y * y;
  float32 z2 = z * z;
  float32 xy = x * y;
  float32 xz = x * z;
  float32 yz = y * z;
  float32 wx = w * x;
  float32 wy = w * y;
  float32 wz = w * z;

  return matrix3(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy),
                 2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx),
                 2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2));
}

}  // namespace base