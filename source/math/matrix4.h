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

#ifndef __MATRIX4_H__
#define __MATRIX4_H__

#include "base.h"
#include "matrix2.h"
#include "matrix3.h"
#include "vector3.h"
#include "vector4.h"

namespace base {

typedef struct matrix4 {
 public:
  // Column Major:
  //
  // 4x4 =  | 0   4   8   12 |
  //        | 1   5   9   13 |
  //        | 2   6   10  14 |
  //        | 3   7   11  15 |

  union {
    struct {
      float32 m00;
      float32 m10;
      float32 m20;
      float32 m30;

      float32 m01;
      float32 m11;
      float32 m21;
      float32 m31;

      float32 m02;
      float32 m12;
      float32 m22;
      float32 m32;

      float32 m03;
      float32 m13;
      float32 m23;
      float32 m33;
    };

    float32 m[16];
  };

 public:
  matrix4();
  matrix4(const matrix2& rhs);
  matrix4(const matrix3& rhs);
  matrix4(const matrix4& rhs);
  matrix4(float32 _m00, float32 _m10, float32 _m20, float32 _m30, float32 _m01,
          float32 _m11, float32 _m21, float32 _m31, float32 _m02, float32 _m12,
          float32 _m22, float32 _m32, float32 _m03, float32 _m13, float32 _m23,
          float32 _m33);
  ~matrix4();

  operator matrix2();
  operator matrix3();

  inline const matrix4& clear();
  inline const matrix4& identity();

  matrix4 inverse() const;
  inline matrix4 transpose() const;
  float32 determinant() const;
  inline matrix4 inverse_transpose() const;

  inline matrix4 translation(float32 tx, float32 ty,
                             float32 tz) const;  // creates a translation matrix
  inline matrix4 scale(float32 sx, float32 sy,
                       float32 sz) const;  // creates a scaling matrix
  inline matrix4 rotation(
      float32 rad, const vector3& axis) const;  // creates a rotation matrix
  inline matrix4 rotation_x(float32 rad) const;
  inline matrix4 rotation_y(float32 rad) const;
  inline matrix4 rotation_z(float32 rad) const;

  inline const matrix4& orient(const vector3& pos, const vector3& xaxis,
                               const vector3& yaxis, const vector3& zaxis);

  inline bool is_diagonal() const;
  inline bool is_invertible() const;

  inline const matrix4& ortho(float32 left, float32 right, float32 top,
                              float32 bottom, float32 nz, float32 fz);
  inline const matrix4& frustum(float32 left, float32 right, float32 bottom,
                                float32 top, float32 nz, float32 fz);
  inline const matrix4& perspective(float32 fovy, float32 aspect, float32 nz,
                                    float32 fz);
  inline const matrix4& look_at(const vector3& origin, const vector3& target,
                                const vector3& up);
  inline const matrix4& look(const vector3& origin, const vector3& view,
                             const vector3& up);
  inline const matrix4& set(const matrix4& rhs);
  inline const matrix4& set(float32 _m00, float32 _m10, float32 _m20,
                            float32 _m30, float32 _m01, float32 _m11,
                            float32 _m21, float32 _m31, float32 _m02,
                            float32 _m12, float32 _m22, float32 _m32,
                            float32 _m03, float32 _m13, float32 _m23,
                            float32 _m33);

  inline const matrix4& operator=(const matrix4& rhs);
  inline const matrix4& operator-=(const matrix4& rhs);
  inline const matrix4& operator+=(const matrix4& rhs);
  inline const matrix4& operator*=(const matrix4& rhs);
  inline const matrix4& operator/=(const matrix4& rhs);

  inline bool operator==(const matrix4& rhs) const;
  inline bool operator!=(const matrix4& rhs) const;
  inline matrix4 operator-(const matrix4& rhs) const;
  inline matrix4 operator+(const matrix4& rhs) const;
  inline matrix4 operator*(const matrix4& rhs) const;
  inline vector4 operator*(const vector4& rhs) const;
  inline matrix4 operator/(const matrix4& rhs) const;

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

} matrix4;

inline bool matrix4::is_diagonal() const {
  for (uint8 i = 0; i < 16; i++) {
    uint8 r = i % 4;
    uint8 c = i / 4;

    if (m[i] != 0 && r != c) return false;
  }

  return true;
}

inline bool matrix4::is_invertible() const { return (0 != determinant()); }

inline const matrix4& matrix4::clear() {
  memset(m, 0, 16 * sizeof(float32));

  return (*this);
}

inline const matrix4& matrix4::set(const matrix4& rhs) { return (*this) = rhs; }

inline const matrix4& matrix4::set(float32 _m00, float32 _m10, float32 _m20,
                                   float32 _m30, float32 _m01, float32 _m11,
                                   float32 _m21, float32 _m31, float32 _m02,
                                   float32 _m12, float32 _m22, float32 _m32,
                                   float32 _m03, float32 _m13, float32 _m23,
                                   float32 _m33) {
  m[0] = _m00;
  m[4] = _m01;
  m[8] = _m02;
  m[12] = _m03;
  m[1] = _m10;
  m[5] = _m11;
  m[9] = _m12;
  m[13] = _m13;
  m[2] = _m20;
  m[6] = _m21;
  m[10] = _m22;
  m[14] = _m23;
  m[3] = _m30;
  m[7] = _m31;
  m[11] = _m32;
  m[15] = _m33;

  return (*this);
}

inline matrix4 matrix4::transpose() const {
  matrix4 out;

  out[0] = m[0];
  out[4] = m[1];
  out[8] = m[2];
  out[12] = m[3];
  out[1] = m[4];
  out[5] = m[5];
  out[9] = m[6];
  out[13] = m[7];
  out[2] = m[8];
  out[6] = m[9];
  out[10] = m[10];
  out[14] = m[11];
  out[3] = m[12];
  out[7] = m[13];
  out[11] = m[14];
  out[15] = m[15];

  return out;
}

inline matrix4 matrix4::inverse_transpose() const {
  matrix4 output = (*this);
  output = output.inverse();
  output = output.transpose();

  return output;
}

inline const matrix4& matrix4::identity() {
  clear();

  m[0] = m[5] = m[10] = m[15] = 1;

  return (*this);
}

inline matrix4 matrix4::translation(float32 tx, float32 ty, float32 tz) const {
  matrix4 output;
  output = output.identity();

  output[12] = tx;
  output[13] = ty;
  output[14] = tz;

  return output;
}

inline matrix4 matrix4::scale(float32 sx, float32 sy, float32 sz) const {
  matrix4 output;
  output = output.identity();

  output[0] = sx;
  output[5] = sy;
  output[10] = sz;

  return output;
}

inline matrix4 matrix4::rotation(float32 rad, const vector3& axis) const {
  float32 x, y, z;
  float32 c = cos(rad);
  float32 s = sin(rad);
  float32 t = 1.0f - c;
  matrix4 output;

  vector3 temp_axis = axis.normalize();

  x = temp_axis[0];
  y = temp_axis[1];
  z = temp_axis[2];

  output[0] = t * x * x + c;
  output[4] = t * x * y - s * z;
  output[8] = t * x * z + s * y;
  output[12] = 0.0f;
  output[1] = t * x * y + s * z;
  output[5] = t * y * y + c;
  output[9] = t * y * z - s * x;
  output[13] = 0.0f;
  output[2] = t * x * z - s * y;
  output[6] = t * y * z + s * x;
  output[10] = t * z * z + c;
  output[14] = 0.0f;
  output[3] = 0.0f;
  output[7] = 0.0f;
  output[11] = 0.0f;
  output[15] = 1.0f;

  return output;
}

inline matrix4 matrix4::rotation_x(float32 rad) const {
  matrix4 output;
  output = output.identity();

  // This produces the same result as rotation( rad, BASE_X_AXIS )
  // without the extra call

  output[5] = cos(rad);
  output[6] = sin(rad);
  output[9] = -sin(rad);
  output[10] = cos(rad);

  return output;
}

inline matrix4 matrix4::rotation_y(float32 rad) const {
  matrix4 output;
  output = output.identity();

  // This produces the same result as rotation( rad, BASE_Y_AXIS )
  // without the extra call

  output[0] = cos(rad);
  output[2] = sin(rad);
  output[8] = -sin(rad);
  output[10] = cos(rad);

  return output;
}

inline matrix4 matrix4::rotation_z(float32 rad) const {
  matrix4 output;
  output = output.identity();

  // This produces the same result as rotation( rad, BASE_Z_AXIS )
  // without the extra call

  output[0] = cos(rad);
  output[1] = sin(rad);
  output[4] = -sin(rad);
  output[5] = cos(rad);

  return output;
}

inline const matrix4& matrix4::orient(const vector3& pos, const vector3& xaxis,
                                      const vector3& yaxis,
                                      const vector3& zaxis) {
  m00 = xaxis.x;
  m10 = xaxis.y;
  m20 = xaxis.z;
  m30 = -xaxis.dot(pos);

  m01 = yaxis.x;
  m11 = yaxis.y;
  m21 = yaxis.z;
  m31 = -yaxis.dot(pos);

  m02 = zaxis.x;
  m12 = zaxis.y;
  m22 = zaxis.z;
  m32 = -zaxis.dot(pos);

  m03 = 0;
  m13 = 0;
  m23 = 0;
  m33 = 1;

  return (*this);
}

inline const matrix4& matrix4::ortho(float32 left, float32 right, float32 top,
                                     float32 bottom, float32 nz, float32 fz) {
  if (BASE_PARAM_CHECK) {
    if (0 == (right - left) || 0 == (top - bottom) || 0 == (fz - nz)) {
      return (*this);
    }
  }

  float32 tx = -1.0 * (right + left) / (right - left);
  float32 ty = -1.0 * (top + bottom) / (top - bottom);
  float32 tz = -1.0 * (fz + nz) / (fz - nz);

  m[0] = 2.0 / (right - left);
  m[4] = 0;
  m[8] = 0;
  m[12] = tx;
  m[1] = 0;
  m[5] = 2.0 / (top - bottom);
  m[9] = 0;
  m[13] = ty;
  m[2] = 0;
  m[6] = 0;
  m[10] = -2.0 / (fz - nz);
  m[14] = tz;
  m[3] = 0;
  m[7] = 0;
  m[11] = 0;
  m[15] = 1.0;

  return (*this);
}

inline const matrix4& matrix4::frustum(float32 left, float32 right,
                                       float32 bottom, float32 top, float32 nz,
                                       float32 fz) {
  m[0] = 2 * nz / (right - left);
  m[5] = 2 * nz / (top - bottom);
  m[8] = (right + left) / (2.0f * nz);
  m[9] = (bottom + top) / 2.0f * nz;
  m[10] = -1 * (fz + nz) / (fz - nz);
  m[11] = -1;
  m[14] = (-2 * nz * fz) / (fz - nz);

  m[1] = m[2] = m[3] = m[4] = m[6] = m[7] = m[12] = m[13] = m[15] = 0;

  return (*this);
}

inline const matrix4& matrix4::perspective(float32 fovy, float32 aspect,
                                           float32 nz, float32 fz) {
  float32 fovx = fovy * aspect;
  float32 width = 1.0 / tanf(fovx * 0.5);
  float32 height = 1.0 / tanf(fovy * 0.5);
  float32 clip_ratio = fz / (fz - nz);

  identity();

  m00 = width;
  m11 = height;
  m22 = clip_ratio;
  m32 = -clip_ratio * nz;
  m23 = 1;

  return (*this);
}

inline const matrix4& matrix4::look(const vector3& origin, const vector3& view,
                                    const vector3& up) {
  // Generate our right vector then create an orientation matrix

  matrix4 output;

  vector3 newView = view * -1.0f;
  vector3 right = up.cross(newView).normalize();
  vector3 newUp = newView.cross(right).normalize();

  orient(origin, right, newUp, newView);

  return (*this);
}

inline const matrix4& matrix4::look_at(const vector3& origin,
                                       const vector3& target,
                                       const vector3& up) {
  // Generate our right vector then create an orientation matrix

  vector3 view = (target - origin).normalize();
  vector3 right = up.cross(view).normalize();
  vector3 newUp = view.cross(right).normalize();
  orient(origin, right, newUp, view);

  return (*this);
}

inline const matrix4& matrix4::operator=(const matrix4& rhs) {
  m[0] = rhs[0];
  m[4] = rhs[4];
  m[8] = rhs[8];
  m[12] = rhs[12];
  m[1] = rhs[1];
  m[5] = rhs[5];
  m[9] = rhs[9];
  m[13] = rhs[13];
  m[2] = rhs[2];
  m[6] = rhs[6];
  m[10] = rhs[10];
  m[14] = rhs[14];
  m[3] = rhs[3];
  m[7] = rhs[7];
  m[11] = rhs[11];
  m[15] = rhs[15];

  return (*this);
}

inline bool matrix4::operator==(const matrix4& rhs) const {
  for (uint8 i = 0; i < 16; i++) {
    if (m[i] != rhs[i]) return false;
  }

  return true;
}

inline bool matrix4::operator!=(const matrix4& rhs) const {
  return !((*this) == rhs);
}

inline matrix4 matrix4::operator-(const matrix4& rhs) const {
  matrix4 output;

  for (uint8 i = 0; i < 16; i++) {
    output[i] = m[i] - rhs[i];
  }

  return output;
}

inline matrix4 matrix4::operator+(const matrix4& rhs) const {
  matrix4 output;

  for (uint8 i = 0; i < 16; i++) {
    output[i] = m[i] + rhs[i];
  }

  return output;
}

inline const matrix4& matrix4::operator-=(const matrix4& rhs) {
  return (*this) = (*this) - rhs;
}

inline const matrix4& matrix4::operator+=(const matrix4& rhs) {
  return (*this) = (*this) + rhs;
}

inline matrix4 matrix4::operator*(const matrix4& rhs) const {
  matrix4 output;

  uint32 r;
  uint32 c;

  for (int32 k = 0; k < 16; k++) {
    r = k % 4;
    c = (int)(k / 4);

    output[k] = 0.0;

    for (int32 i = 0; i < 4; i++)
      output[k] += m[r + (i * 4)] * rhs[(c * 4) + i];
  }

  return output;
}

inline const matrix4& matrix4::operator*=(const matrix4& rhs) {
  return ((*this) = (*this) * rhs);
}

inline vector4 matrix4::operator*(const vector4& rhs) const {
  return vector4(rhs.x * m[0] + rhs.y * m[4] + rhs.z * m[8] + rhs.w * m[12],
                 rhs.x * m[1] + rhs.y * m[5] + rhs.z * m[9] + rhs.w * m[13],
                 rhs.x * m[2] + rhs.y * m[6] + rhs.z * m[10] + rhs.w * m[14],
                 rhs.x * m[3] + rhs.y * m[7] + rhs.z * m[11] + rhs.w * m[15]);
}

inline matrix4 matrix4::operator/(const matrix4& rhs) const {
  if (BASE_PARAM_CHECK) {
    for (uint32 i = 0; i < 16; i++) {
      if (0 == rhs[i]) {
        return matrix4();
      }
    }
  }

  matrix4 temp;

  for (uint32 i = 0; i < 16; i++) {
    temp[i] = 1.0f / rhs[i];
  }

  return (*this) * temp;
}

inline const matrix4& matrix4::operator/=(const matrix4& rhs) {
  return (*this) = (*this) / rhs;
}

inline float32& matrix4::operator[](int32 i) {
  if (i > 15) return m[0];

  return m[i];
}

inline const float32& matrix4::operator[](int32 i) const {
  if (i > 15) return m[0];

  return m[i];
}

}  // namespace base

#endif  // __MATRIX4_H__
