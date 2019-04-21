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

#ifndef __MATRIX2_H__
#define __MATRIX2_H__

#include "base.h"
#include "solver.h"
#include "vector2.h"

namespace base {

typedef struct matrix2 {
 public:
  // Column Major:
  //
  // 2x2 =  | 0  2 |
  //        | 1  3 |

  union {
    struct {
      float32 m00;
      float32 m10;
      float32 m01;
      float32 m11;
    };

    float32 m[4];
  };

 public:
  matrix2();
  matrix2(const matrix2& rhs);
  matrix2(float32 _m00, float32 _m10, float32 _m01, float32 _m11);
  ~matrix2();

  inline const matrix2& clear();
  inline const matrix2& identity();

  inline matrix2 inverse() const;
  inline matrix2 transpose() const;
  inline float32 determinant() const;
  inline matrix2 inverse_transpose() const;
  inline matrix2 scale(float32 sx,
                       float32 sy) const;      // creates a scaling matrix
  inline matrix2 rotation(float32 rad) const;  // creates a rotation matrix

  inline float32 trace() const;
  inline vector2 eigenvalues() const;
  inline matrix2 eigenvectors() const;

  inline bool is_diagonal() const;
  inline bool is_invertible() const;

  inline const matrix2& set(matrix2& rhs);
  inline const matrix2& set(float32 _m00, float32 _m10, float32 _m01,
                            float32 _m11);

  inline const matrix2& operator=(const matrix2& rhs);
  inline const matrix2& operator-=(const matrix2& rhs);
  inline const matrix2& operator+=(const matrix2& rhs);
  inline const matrix2& operator*=(const matrix2& rhs);  // matrix concatenation
  inline const matrix2& operator/=(const matrix2& rhs);  // 1/rhs concatentation

  inline bool operator==(const matrix2& rhs) const;
  inline bool operator!=(const matrix2& rhs) const;
  inline matrix2 operator-(const matrix2& rhs) const;
  inline matrix2 operator+(const matrix2& rhs) const;
  inline matrix2 operator*(const matrix2& rhs) const;
  inline vector2 operator*(const vector2& rhs) const;
  inline matrix2 operator*(float32 rhs) const;
  inline matrix2 operator/(const matrix2& rhs) const;

  // Here we define two variants of our bracket operator. The first
  // operator handles the lvalue case:
  //
  //     matrix[0] = ...
  //
  // The second operator passes a const this pointer, and is useful
  // when operating on const references (e.g. any of our other
  // operators that receive const matrix3 & rhs as a parameter.)

  inline float32& operator[](int32 i);
  inline const float32& operator[](int32 i) const;

} matrix2;

inline float32 matrix2::trace() const { return m[0] + m[3]; }

inline vector2 matrix2::eigenvalues() const {
  // Solve det( M - lamda * Identity ) = 0
  float32 a = 1;
  float32 b = -trace();
  float32 c = m00 * m11 - m01 * m10;

  vector2 output;

  BASE_SOLVER_SOLUTION solution = {0};
  solve_quadratic(a, b, c, &solution);

  output.x = solution.t[0];
  output.y = solution.t[1];

  return output;
}

inline matrix2 matrix2::eigenvectors() const {
  // Solve ( M - lamda * Identity ) * Vi = 0
  // We should have one eigenvector per eigen value

  matrix2 output;
  vector2 values = eigenvalues();

  matrix2 i = (*this);
  matrix2 m = (*this);

  i.identity();

  // First eigen value calculation

  matrix2 o1 = m - i * values.x;

  //      o1.m[0] * x + o1.m[2] * y = 0
  //      o1.m[1] * x + o1.m[3] * y = 0
  //
  //  => Since this vector expresses only direction, and not magnitude,
  //     we can set x = 1 to solve for y.
  //
  //  => We reduce using this setting:
  //  => o1.m[0] - o1.m[1] = o1.m[3] * y - o1.m[2] * y
  //  => y = ( o1.m[0] - o1.m[1] ) / ( o1.m[3] - o1.m[2] )
  //
  //  Thus, we've solved for [ x, y ]

  float32 a1 = (o1.m[0] - o1.m[1]);
  float32 b1 = (o1.m[3] - o1.m[2]);

  output.m[0] = 1;

  if (0.0 != b1)
    output.m[2] = a1 / b1;
  else
    output.m[2] = -o1.m[1];

  // Second eigen value calculation

  matrix2 o2 = m - i * values.y;

  float32 a2 = (o2.m[0] - o2.m[1]);
  float32 b2 = (o2.m[3] - o2.m[2]);

  output.m[1] = 1;

  if (0.0 != b2)
    output.m[3] = a2 / b2;
  else
    output.m[3] = -o2.m[1];

  return output;
}

inline bool matrix2::is_diagonal() const {
  // A diagonal matrix has non-zero values only along the
  // diagonal, and zero everywhere else.
  if (m10 || m01) return false;

  return true;
}

inline bool matrix2::is_invertible() const { return (0 != determinant()); }

inline matrix2 const& matrix2::clear() {
  memset(m, 0, 4 * sizeof(float32));

  return (*this);
}

inline matrix2 const& matrix2::identity() {
  m[0] = m[3] = 1;
  m[1] = m[2] = 0;

  return (*this);
}

inline matrix2 matrix2::transpose() const {
  matrix2 output;

  output[0] = m[0];
  output[3] = m[3];

  output[1] = m[2];
  output[2] = m[1];

  return output;
}

inline float32 matrix2::determinant() const {
  return (m[0] * m[3]) - (m[1] * m[2]);
}

inline matrix2 matrix2::inverse() const {
  matrix2 output;
  float32 inv_det = 0;
  float32 det = determinant();

  if (0 == det) {
    return matrix2();
  }

  inv_det = 1.0f / det;

  output[0] = inv_det * m[3];
  output[1] = inv_det * (-1.0f * m[1]);
  output[2] = inv_det * (-1.0f * m[2]);
  output[3] = inv_det * m[0];

  return output;
}

inline matrix2 matrix2::scale(float32 sx, float32 sy) const {
  matrix2 output;

  output[0] = sx;
  output[1] = 0.0f;
  output[2] = 0.0f;
  output[3] = sy;

  return output;
}

inline matrix2 matrix2::rotation(float32 rad) const {
  matrix2 output;

  output[0] = cos(rad);
  output[1] = sin(rad);
  output[2] = -sin(rad);
  output[3] = cos(rad);

  return output;
}

inline matrix2 matrix2::inverse_transpose() const {
  matrix2 output = (*this);

  output = output.inverse();
  output = output.transpose();

  return output;
}

inline const matrix2& matrix2::set(matrix2& rhs) { return (*this) = rhs; }

inline const matrix2& matrix2::set(float32 _m00, float32 _m10, float32 _m01,
                                   float32 _m11) {
  m[0] = _m00;
  m[1] = _m10;
  m[2] = _m01;
  m[3] = _m11;

  return (*this);
}

inline const matrix2& matrix2::operator=(const matrix2& rhs) {
  m[0] = rhs[0];
  m[1] = rhs[1];
  m[2] = rhs[2];
  m[3] = rhs[3];

  return (*this);
}

inline bool matrix2::operator==(const matrix2& rhs) const {
  return (m[0] == rhs[0] && m[1] == rhs[1] && m[2] == rhs[2] && m[3] == rhs[3]);
}

inline bool matrix2::operator!=(const matrix2& rhs) const {
  return !((*this) == rhs);
}

inline matrix2 matrix2::operator-(const matrix2& rhs) const {
  matrix2 output;

  for (unsigned char i = 0; i < 4; i++) {
    output[i] = m[i] - rhs[i];
  }

  return output;
}

inline matrix2 matrix2::operator+(const matrix2& rhs) const {
  matrix2 output;

  for (unsigned char i = 0; i < 4; i++) {
    output[i] = m[i] + rhs[i];
  }

  return output;
}

inline const matrix2& matrix2::operator-=(const matrix2& rhs) {
  for (uint8 i = 0; i < 4; i++) {
    m[i] = m[i] - rhs[i];
  }

  return (*this);
}

inline const matrix2& matrix2::operator+=(const matrix2& rhs) {
  for (uint8 i = 0; i < 4; i++) {
    m[i] = m[i] + rhs[i];
  }

  return (*this);
}

inline matrix2 matrix2::operator*(const matrix2& rhs) const {
  matrix2 output;

  output[0] = rhs[0] * m[0] + rhs[1] * m[2];
  output[1] = rhs[0] * m[1] + rhs[1] * m[3];
  output[2] = rhs[2] * m[0] + rhs[3] * m[2];
  output[3] = rhs[2] * m[1] + rhs[3] * m[3];

  return output;
}

inline const matrix2& matrix2::operator*=(const matrix2& rhs) {
  return ((*this) = (*this) * rhs);
}

inline vector2 matrix2::operator*(const vector2& rhs) const {
  return vector2(rhs.x * m[0] + rhs.y * m[2], rhs.x * m[1] + rhs.y * m[3]);
}

inline matrix2 matrix2::operator*(float32 rhs) const {
  matrix2 out = (*this);

  for (uint8 i = 0; i < 4; i++) {
    out.m[i] *= rhs;
  }

  return out;
}

inline matrix2 matrix2::operator/(const matrix2& rhs) const {
  if (BASE_PARAM_CHECK) {
    for (uint32 i = 0; i < 4; i++) {
      if (0 == rhs[i]) {
        return matrix2();
      }
    }
  }

  matrix2 temp;

  for (uint32 i = 0; i < 4; i++) {
    temp[i] = 1.0f / rhs[i];
  }

  return (*this) * temp;
}

inline const matrix2& matrix2::operator/=(const matrix2& rhs) {
  return (*this) = (*this) / rhs;
}

inline float32& matrix2::operator[](int32 i) {
  if (i > 3) return m[0];

  return m[i];
}

inline float32 const& matrix2::operator[](int32 i) const {
  if (i > 3) return m[0];

  return m[i];
}

}  // namespace base

#endif  // __MATRIX2_H__
