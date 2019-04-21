/*
//
// Copyright (c) 1998-2019 Joe Bertolami. All Right Reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//   AND ANY EXPRESS OR IMPLIED WARRANTIES, CLUDG, BUT NOT LIMITED TO, THE
//   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//   ARE DISCLAIMED.  NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//   LIABLE FOR ANY DIRECT, DIRECT, CIDENTAL, SPECIAL, EXEMPLARY, OR
//   CONSEQUENTIAL DAMAGES (CLUDG, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSESS TERRUPTION)
//   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER  CONTRACT, STRICT
//   LIABILITY, OR TORT (CLUDG NEGLIGENCE OR OTHERWISE) ARISG  ANY WAY  OF THE
//   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Additional Information:
//
//   For more information, visit http://www.bertolami.com.
//
*/

#ifndef __MATRIX3_H__
#define __MATRIX3_H__

#include "base.h"
#include "matrix2.h"
#include "solver.h"
#include "vector3.h"

namespace base {

typedef struct matrix3 {
 public:
  // Column Major:
  //
  // 3x3 =  | 0  3  6 |
  //        | 1  4  7 |
  //        | 2  5  8 |

  union {
    struct {
      float32 m00;
      float32 m10;
      float32 m20;

      float32 m01;
      float32 m11;
      float32 m21;

      float32 m02;
      float32 m12;
      float32 m22;
    };

    float32 m[9];
  };

 public:
  matrix3();
  matrix3(const matrix2& rhs);
  matrix3(const matrix3& rhs);
  matrix3(float32 _m00, float32 _m10, float32 _m20, float32 _m01, float32 _m11,
          float32 _m21, float32 _m02, float32 _m12, float32 _m22);
  ~matrix3();

  operator matrix2();

  inline const matrix3& clear();
  inline const matrix3& identity();

  inline matrix3 inverse() const;
  inline matrix3 transpose() const;
  inline float32 determinant() const;
  inline matrix3 inverse_transpose() const;

  inline float32 trace() const;
  inline vector3 eigenvalues() const;
  inline matrix3 eigenvectors() const;

  inline bool is_diagonal() const;
  inline bool is_invertible() const;

  float32 minor(uint8 i, uint8 j) const;
  inline float32 cofactor(uint8 i, uint8 j) const;

  inline matrix3 scale(float32 sx, float32 sy,
                       float32 sz) const;  // creates a scaling matrix
  inline matrix3 rotation(
      float32 rad, const vector3& axis) const;  // creates a rotation matrix
  inline matrix3 rotation_x(float32 rad) const;
  inline matrix3 rotation_y(float32 rad) const;
  inline matrix3 rotation_z(float32 rad) const;

  inline const matrix3& orient(const vector3& xaxis, const vector3& yaxis,
                               const vector3& zaxis);

  inline const matrix3& set(matrix3& rhs);
  inline const matrix3& set(float32 _m00, float32 _m10, float32 _m20,
                            float32 _m01, float32 _m11, float32 _m21,
                            float32 _m02, float32 _m12, float32 _m22);

  inline const matrix3& operator=(const matrix3& rhs);
  inline const matrix3& operator-=(const matrix3& rhs);
  inline const matrix3& operator+=(const matrix3& rhs);
  inline const matrix3& operator*=(const matrix3& rhs);
  inline const matrix3& operator/=(const matrix3& rhs);

  inline bool operator==(const matrix3& rhs) const;
  inline bool operator!=(const matrix3& rhs) const;
  inline matrix3 operator-(const matrix3& rhs) const;
  inline matrix3 operator+(const matrix3& rhs) const;
  inline matrix3 operator*(const matrix3& rhs) const;
  inline vector3 operator*(const vector3& rhs) const;
  inline matrix3 operator*(float32 rhs) const;
  inline matrix3 operator/(const matrix3& rhs) const;

  // Here we define two variants of our bracket operator. The first
  // operator handles the lvalue case:
  //
  //     matrix[0] = ...
  //
  // The second operator passes a const this pointer, and is useful
  // when operating on const references (e.g. any of our other
  // operators that receive const CMatrix3 & rhs as a parameter.)

  inline float32& operator[](int32 i);
  inline const float32& operator[](int32 i) const;

} matrix3;

inline float32 matrix3::cofactor(uint8 i, uint8 j) const {
  if (0 == ((i + j) % 2)) {
    // Even -- our base is positive

    return minor(i, j);
  }

  // Odd -- our base is negative

  return -1.0f * minor(i, j);
}

inline float32 matrix3::trace() const { return (m[0] + m[4] + m[8]); }

inline bool matrix3::is_diagonal() const {
  for (uint8 i = 0; i < 9; i++) {
    uint8 r = i % 3;
    uint8 c = i / 3;

    if (m[i] != 0 && r != c) return false;
  }

  return true;
}

inline bool matrix3::is_invertible() const { return (0 != determinant()); }

inline vector3 matrix3::eigenvalues() const {
  // Solve det( M - lamda * Identity ) = 0
  // Determinant of a 3xx3:
  //
  //   | a  b  c |
  //   | d  e  f | => aei + bfg + cdh - ceg - bdi - afh
  //   | g  h  i |
  //
  // The characteristic polynomial is defined as:
  //
  // (a - lambda)(e - lambda)(i - lambda) + bfg + cdh - c(e - lambda)g - bd(i -
  // lambda) - (a - lambda)fh
  //
  //    or
  //
  // -lambda^3 + trace() * lambda ^ 2 - c2 * lambda + determinant()

  float32 c2 = 0.0f;

  // c2 is the sum of the principal minors of our matrix

  for (uint8 i = 0; i < 3; i++) {
    c2 += minor(i, i);
  }

  float32 a = -1;
  float32 b = trace();
  float32 c = -c2;
  float32 d = determinant();

  vector3 output;

  BASE_SOLVER_SOLUTION solution = {0};

  solve_cubic(a, b, c, d, &solution);

  output.x = solution.t[0];
  output.y = solution.t[1];
  output.z = solution.t[2];

  return output;
}

inline matrix3 matrix3::eigenvectors() const {
  // Solve det( M - lamda * Identity ) * Vi = 0
  // We should have one eigenvector per eigen value

  matrix3 output;
  vector3 values = eigenvalues();

  matrix3 i = (*this);
  matrix3 m = (*this);

  i.identity();

  // z = ( m[1] * m[3] - m[4] * ( -1.0 * m[0] ) ) / ( m[7] * m[3] - m[4] * (
  // -1.0 * m[6] ) ) y = ( -1.0 * m[0] - m[6] * z ) / m[3] x = 1
  //
  // First eigen value calculation

  matrix3 o1 = m - i * values.x;

  float32 z1_denom = (o1.m[7] * o1.m[3] - o1.m[4] * (-1.0 * o1.m[6]));
  float32 z1 = (o1.m[1] * o1.m[3] - o1.m[4] * (-1.0 * o1.m[0])) / z1_denom;
  float32 y1 = (-1.0 * o1.m[0] - o1.m[6] * z1) / o1.m[3];
  float32 x1 = 1.0;

  output.m[0] = x1;
  output.m[3] = y1;
  output.m[6] = z1;

  // Second eigen value calculation

  matrix3 o2 = m - i * values.y;

  float32 z2_denom = (o2.m[7] * o2.m[3] - o2.m[4] * (-1.0 * o2.m[6]));
  float32 z2 = (o2.m[1] * o2.m[3] - o2.m[4] * (-1.0 * o2.m[0])) / z2_denom;
  float32 y2 = (-1.0 * o2.m[0] - o2.m[6] * z2) / o2.m[3];
  float32 x2 = 1.0;

  output.m[1] = x2;
  output.m[4] = y2;
  output.m[7] = z2;

  // Third eigen value calculation

  matrix3 o3 = m - i * values.z;

  float32 z3_denom = (o3.m[7] * o3.m[3] - o3.m[4] * (-1.0 * o3.m[6]));
  float32 z3 = (o3.m[1] * o3.m[3] - o3.m[4] * (-1.0 * o3.m[0])) / z3_denom;
  float32 y3 = (-1.0 * o3.m[0] - o3.m[6] * z3) / o3.m[3];
  float32 x3 = 1.0;

  output.m[2] = x3;
  output.m[5] = y3;
  output.m[8] = z3;

  return output;
}

inline const matrix3& matrix3::clear() {
  memset(m, 0, 9 * sizeof(float32));

  return (*this);
}

inline const matrix3& matrix3::set(matrix3& rhs) { return (*this) = rhs; }

inline const matrix3& matrix3::set(float32 _m00, float32 _m10, float32 _m20,
                                   float32 _m01, float32 _m11, float32 _m21,
                                   float32 _m02, float32 _m12, float32 _m22) {
  m[0] = _m00;
  m[3] = _m01;
  m[6] = _m02;
  m[1] = _m10;
  m[4] = _m11;
  m[7] = _m12;
  m[2] = _m20;
  m[5] = _m21;
  m[8] = _m22;

  return (*this);
}

inline const matrix3& matrix3::identity() {
  m[0] = m[3] = m[6] = 0;
  m[1] = m[4] = m[7] = 0;
  m[2] = m[5] = m[8] = 0;
  m[0] = m[4] = m[8] = 1.0;

  return (*this);
}

inline matrix3 matrix3::transpose() const {
  matrix3 output;

  output[0] = m[0];
  output[3] = m[1];
  output[6] = m[2];
  output[1] = m[3];
  output[4] = m[4];
  output[7] = m[5];
  output[2] = m[6];
  output[5] = m[7];
  output[8] = m[8];

  return output;
}

inline float32 matrix3::determinant() const {
  return m[0] * (m[4] * m[8] - m[5] * m[7]) -
         m[3] * (m[1] * m[8] - m[2] * m[7]) +
         m[6] * (m[1] * m[5] - m[4] * m[2]);
}

inline matrix3 matrix3::inverse() const {
  matrix3 adj;
  matrix3 adjT;
  matrix3 output;
  float32 inv_det = 0;
  float32 det = determinant();

  if (0 == det) {
    return matrix3();
  }

  inv_det = 1.0f / det;

  // calculate adjoint matrix then transpose it

  adj[0] = (m[4] * m[8] - m[5] * m[7]);
  adj[1] = -1 * (m[3] * m[8] - m[5] * m[6]);
  adj[2] = (m[3] * m[7] - m[6] * m[4]);

  adj[3] = -1 * (m[1] * m[8] - m[2] * m[7]);
  adj[4] = (m[0] * m[8] - m[6] * m[2]);
  adj[5] = -1 * (m[0] * m[7] - m[6] * m[1]);

  adj[6] = (m[1] * m[5] - m[2] * m[4]);
  adj[7] = -1 * (m[0] * m[5] - m[3] * m[2]);
  adj[8] = (m[0] * m[4] - m[1] * m[3]);

  adjT = adj.transpose();

  // now multiply all values by inv_det

  for (uint32 i = 0; i < 9; i++) output[i] = adjT[i] * inv_det;

  return output;
}

inline matrix3 matrix3::scale(float32 sx, float32 sy, float32 sz) const {
  matrix3 output;

  output = output.identity();

  output[0] = sx;
  output[4] = sy;
  output[8] = sz;

  return output;
}

inline matrix3 matrix3::rotation(float32 rad, const vector3& axis) const {
  float32 x, y, z;
  float32 c = cos(rad);
  float32 s = sin(rad);
  float32 t = 1.0f - c;
  matrix3 output;

  vector3 temp_axis = axis.normalize();

  x = temp_axis[0];
  y = temp_axis[1];
  z = temp_axis[2];

  output[0] = t * x * x + c;
  output[3] = t * x * y - s * z;
  output[6] = t * x * z + s * y;
  output[1] = t * x * y + s * z;
  output[4] = t * y * y + c;
  output[7] = t * y * z - s * x;
  output[2] = t * x * z - s * y;
  output[5] = t * y * z + s * x;
  output[8] = t * z * z + c;

  return output;
}

inline matrix3 matrix3::rotation_x(float32 rad) const {
  matrix3 output;

  output = output.identity();

  // This produces the same result as rotation( rad, BASE_X_AXIS )
  // without the extra call

  output[4] = cos(rad);
  output[5] = sin(rad);
  output[7] = -sin(rad);
  output[8] = cos(rad);

  return output;
}

inline matrix3 matrix3::rotation_y(float32 rad) const {
  matrix3 output;

  output = output.identity();

  // This produces the same result as rotation( rad, BASE_Y_AXIS )
  // without the extra call

  output[0] = cos(rad);
  output[2] = sin(rad);
  output[6] = -sin(rad);
  output[8] = cos(rad);

  return output;
}

inline matrix3 matrix3::rotation_z(float32 rad) const {
  matrix3 output;

  output = output.identity();

  // This produces the same result as rotation( rad, BASE_Z_AXIS )
  // without the extra call

  output[0] = cos(rad);
  output[1] = sin(rad);
  output[3] = -sin(rad);
  output[4] = cos(rad);

  return output;
}

inline const matrix3& matrix3::orient(const vector3& xaxis,
                                      const vector3& yaxis,
                                      const vector3& zaxis) {
  m[0] = xaxis[0];
  m[3] = xaxis[1];
  m[6] = xaxis[2];
  m[1] = yaxis[0];
  m[4] = yaxis[1];
  m[7] = yaxis[2];
  m[2] = zaxis[0];
  m[5] = zaxis[1];
  m[8] = zaxis[2];

  return (*this);
}

inline matrix3 matrix3::inverse_transpose() const {
  matrix3 output = (*this);
  output = output.inverse();
  output = output.transpose();

  return output;
}

inline const matrix3& matrix3::operator=(const matrix3& rhs) {
  m[0] = rhs[0];
  m[3] = rhs[3];
  m[6] = rhs[6];
  m[1] = rhs[1];
  m[4] = rhs[4];
  m[7] = rhs[7];
  m[2] = rhs[2];
  m[5] = rhs[5];
  m[8] = rhs[8];

  return (*this);
}

inline bool matrix3::operator==(const matrix3& rhs) const {
  for (unsigned char i = 0; i < 9; i++) {
    if (m[i] != rhs[i]) return false;
  }

  return true;
}

inline bool matrix3::operator!=(const matrix3& rhs) const {
  return !((*this) == rhs);
}

inline matrix3 matrix3::operator-(const matrix3& rhs) const {
  matrix3 output;

  for (unsigned char i = 0; i < 9; i++) {
    output[i] = m[i] - rhs[i];
  }

  return output;
}

inline matrix3 matrix3::operator+(const matrix3& rhs) const {
  matrix3 output;

  for (unsigned char i = 0; i < 9; i++) {
    output[i] = m[i] + rhs[i];
  }

  return output;
}

inline const matrix3& matrix3::operator-=(const matrix3& rhs) {
  for (uint8 i = 0; i < 9; i++) {
    m[i] = m[i] - rhs[i];
  }

  return (*this);
}

inline const matrix3& matrix3::operator+=(const matrix3& rhs) {
  for (uint8 i = 0; i < 9; i++) {
    m[i] = m[i] + rhs[i];
  }

  return (*this);
}

inline matrix3 matrix3::operator*(const matrix3& rhs) const {
  matrix3 output;

  uint32 r;
  uint32 c;

  for (uint32 k = 0; k < 9; k++) {
    r = k % 3;
    c = (int32)(k / 3);

    output[k] = 0.0;

    for (int32 i = 0; i < 3; i++) {
      output[k] += m[r + (i * 3)] * rhs[(c * 3) + i];
    }
  }

  return output;
}

inline matrix3 matrix3::operator*(float32 rhs) const {
  matrix3 out = (*this);

  for (uint8 i = 0; i < 9; i++) {
    out.m[i] *= rhs;
  }

  return out;
}

inline const matrix3& matrix3::operator*=(const matrix3& rhs) {
  return ((*this) = (*this) * rhs);
}

inline vector3 matrix3::operator*(const vector3& rhs) const {
  return vector3(rhs.x * m[0] + rhs.y * m[3] + rhs.z * m[6],
                 rhs.x * m[1] + rhs.y * m[4] + rhs.z * m[7],
                 rhs.x * m[2] + rhs.y * m[5] + rhs.z * m[8]);
}

inline matrix3 matrix3::operator/(const matrix3& rhs) const {
  if (BASE_PARAM_CHECK) {
    for (uint32 i = 0; i < 9; i++) {
      if (0 == rhs[i]) {
        return matrix3();
      }
    }
  }

  matrix3 temp;

  for (uint32 i = 0; i < 9; i++) {
    temp[i] = 1.0f / rhs[i];
  }

  return (*this) * temp;
}

inline const matrix3& matrix3::operator/=(const matrix3& rhs) {
  return (*this) = (*this) / rhs;
}

inline float32& matrix3::operator[](int32 i) {
  if (i > 8) return m[0];

  return m[i];
}

inline const float32& matrix3::operator[](int32 i) const {
  if (i > 8) return m[0];

  return m[i];
}

}  // namespace base

#endif  // __MATRIX3_H__
