
#include "matrix2.h"
#include "matrix3.h"

namespace base {

matrix3::matrix3() {
  m[0] = m[3] = m[6] = 0;
  m[1] = m[4] = m[7] = 0;
  m[2] = m[5] = m[8] = 0;
}

matrix3::matrix3(const matrix2& rhs) {
  clear();

  m[0] = rhs[0];
  m[1] = rhs[1];
  m[3] = rhs[2];
  m[4] = rhs[3];
}

matrix3::matrix3(const matrix3& rhs) { (*this) = rhs; }

matrix3::matrix3(float32 _m00, float32 _m10, float32 _m20, float32 _m01,
                 float32 _m11, float32 _m21, float32 _m02, float32 _m12,
                 float32 _m22) {
  set(_m00, _m10, _m20, _m01, _m11, _m21, _m02, _m12, _m22);
}

matrix3::~matrix3() {}

matrix3::operator matrix2() {
  matrix2 output;

  output.identity();

  output[0] = m[0];
  output[1] = m[1];
  output[2] = m[3];
  output[3] = m[4];

  return output;
}

float32 matrix3::minor(uint8 i, uint8 j) const {
  // Remove the ith row, and the jth column, and compute
  // the remaining 2x2 matrix determinant.

  matrix2 o;

  uint8 coord = 0;

  for (uint8 k = 0; k < 9; k++) {
    uint8 r = k % 3;
    uint8 c = k / 3;

    if (r != i && c != j) {
      o.m[coord] = m[k];
      coord++;
    }
  }

  return o.determinant();
}

}  // namespace base