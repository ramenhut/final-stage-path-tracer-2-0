
#include "matrix3.h"
#include "matrix4.h"

namespace base {

matrix4::matrix4() {
  m[0] = m[4] = m[8] = m[12] = 0;
  m[1] = m[5] = m[9] = m[13] = 0;
  m[2] = m[6] = m[10] = m[14] = 0;
  m[3] = m[7] = m[11] = m[15] = 0;
}

matrix4::matrix4(const matrix4& rhs) { (*this) = rhs; }

matrix4::~matrix4() {}

matrix4::matrix4(const matrix2& rhs) {
  clear();

  m[0] = rhs[0];
  m[1] = rhs[1];

  m[4] = rhs[2];
  m[5] = rhs[3];
}

matrix4::matrix4(const matrix3& rhs) {}

matrix4::matrix4(float32 _m00, float32 _m10, float32 _m20, float32 _m30,
                 float32 _m01, float32 _m11, float32 _m21, float32 _m31,
                 float32 _m02, float32 _m12, float32 _m22, float32 _m32,
                 float32 _m03, float32 _m13, float32 _m23, float32 _m33) {
  set(_m00, _m10, _m20, _m30, _m01, _m11, _m21, _m31, _m02, _m12, _m22, _m32,
      _m03, _m13, _m23, _m33);
}

matrix4::operator matrix2() {
  matrix2 out;

  out[0] = m[0];
  out[1] = m[1];
  out[2] = m[4];
  out[3] = m[5];

  return out;
}

matrix4::operator matrix3() {
  matrix3 out;

  out[0] = m[0];
  out[1] = m[1];
  out[2] = m[2];

  out[3] = m[4];
  out[4] = m[5];
  out[5] = m[6];

  out[6] = m[8];
  out[7] = m[9];
  out[8] = m[10];

  return out;
}

float32 matrix4::determinant() const {
  matrix3 one, two, three, four;
  float32 detone, dettwo, detthree, detfour;

  one[0] = m[5];
  one[3] = m[9];
  one[6] = m[13];
  one[1] = m[6];
  one[4] = m[10];
  one[7] = m[14];
  one[2] = m[7];
  one[5] = m[11];
  one[8] = m[15];

  two[0] = m[1];
  two[3] = m[9];
  two[6] = m[13];
  two[1] = m[2];
  two[4] = m[10];
  two[7] = m[14];
  two[2] = m[3];
  two[5] = m[11];
  two[8] = m[15];

  three[0] = m[1];
  three[3] = m[5];
  three[6] = m[13];
  three[1] = m[2];
  three[4] = m[6];
  three[7] = m[14];
  three[2] = m[3];
  three[5] = m[7];
  three[8] = m[15];

  four[0] = m[1];
  four[3] = m[5];
  four[6] = m[9];
  four[1] = m[2];
  four[4] = m[6];
  four[7] = m[10];
  four[2] = m[3];
  four[5] = m[7];
  four[8] = m[11];

  detone = one.determinant();
  dettwo = two.determinant();
  detthree = three.determinant();
  detfour = four.determinant();

  return m[0] * detone - m[4] * dettwo + m[9] * detthree - m[12] * detfour;
}

matrix4 matrix4::inverse() const {
  matrix4 adj;
  matrix3 tmpmat;
  matrix4 adjT;
  matrix4 output;

  float32 inv_det = 0;
  float32 det = determinant();

  if (0.0 == det) {
    return matrix4();
  }

  inv_det = 1.0f / det;

  for (int32 i = 0; i < 4; i++) {
    for (int32 j = 0; j < 4; j++) {
      int32 index = j * 4 + i;

      float32 cofact[3][3];

      int32 ci = 0, cj = 0;

      // calculate cofactor determinant
      for (int32 k = 0; k < 4; k++) {
        if (k == j) continue;

        cj = 0;

        for (int32 n = 0; n < 4; n++) {
          if (n == i) continue;
          cofact[ci][cj] = m[k * 4 + n];
          cj++;
        }

        ci++;
      }

      // calculate cofactor for each
      tmpmat[0] = cofact[0][0];
      tmpmat[3] = cofact[0][1];
      tmpmat[6] = cofact[0][2];
      tmpmat[1] = cofact[1][0];
      tmpmat[4] = cofact[1][1];
      tmpmat[7] = cofact[1][2];
      tmpmat[2] = cofact[2][0];
      tmpmat[5] = cofact[2][1];
      tmpmat[8] = cofact[2][2];

      adj[index] = powf(-1, i + j) * tmpmat.determinant();
    }
  }

  adjT = adj.transpose();

  for (int32 i = 0; i < 16; i++) output[i] = adjT[i] * inv_det;

  return output;
}

}  // namespace base