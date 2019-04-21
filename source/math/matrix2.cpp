
#include "matrix2.h"

namespace base {

matrix2::matrix2() {
  m[0] = m[2] = 0;
  m[1] = m[3] = 0;
}

matrix2::matrix2(const matrix2& rhs) { (*this) = rhs; }

matrix2::matrix2(float32 _m00, float32 _m10, float32 _m01, float32 _m11) {
  set(_m00, _m10, _m01, _m11);
}

matrix2::~matrix2() {}

}  // namespace base