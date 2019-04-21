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

#ifndef __VECTOR4_H__
#define __VECTOR4_H__

#include "base.h"
#include "scalar.h"
#include "vector2.h"
#include "vector3.h"

namespace base {

typedef struct vector4 {
 public:
  union {
    struct {
      float32 x;
      float32 y;
      float32 z;
      float32 w;
    };

    struct {
      float32 r;
      float32 g;
      float32 b;
      float32 a;
    };

    float32 v[4];
  };

 public:
  vector4();
  vector4(const vector2& rhs);
  vector4(const vector3& rhs);
  vector4(const vector4& rhs);
  vector4(float32 xj, float32 yj, float32 zj, float32 wj);
  ~vector4();

  operator vector2();
  operator vector3();

  inline const vector4& clear();
  inline vector4 normalize() const;
  inline vector4 clamp(float32 lower, float32 upper) const;
  inline const vector4& set(const vector4& rhs);
  inline const vector4& set(float32 xj, float32 yj, float32 zj, float32 wj);
  inline vector4 project(const vector4& rhs) const;

  inline bool parallel(const vector4& rhs) const;
  inline bool orthogonal(const vector4& rhs) const;

  inline float32 angle(const vector4& rhs) const;
  inline float32 dot(const vector4& rhs) const;
  inline float32 distance(const vector4& rhs) const;
  inline float32 length() const;

  inline vector4 rotate_x(float32 rad) const;
  inline vector4 rotate_y(float32 rad) const;
  inline vector4 rotate_z(float32 rad) const;
  inline vector4 rotate(float32 rad, const vector3& axis) const;

  const vector4& operator=(const vector2& rhs);
  const vector4& operator=(const vector3& rhs);
  inline const vector4& operator=(const vector4& rhs);

  inline const vector4& operator-=(const vector4& rhs);
  inline const vector4& operator+=(const vector4& rhs);
  inline const vector4& operator*=(const vector4& rhs);
  inline const vector4& operator*=(float32 rhs);
  inline const vector4& operator/=(const vector4& rhs);
  inline const vector4& operator/=(float32 rhs);

  inline bool operator==(const vector4& rhs) const;
  inline bool operator!=(const vector4& rhs) const;
  inline vector4 operator-(const vector4& rhs) const;
  inline vector4 operator+(const vector4& rhs) const;
  inline vector4 operator*(const vector4& rhs) const;
  inline vector4 operator*(float32 rhs) const;
  inline vector4 operator/(const vector4& rhs) const;
  inline vector4 operator/(float32 rhs) const;

  // Here we define two variants of our bracket operator. The first
  // operator handles the lvalue case:
  //
  //     vector[0] = ...
  //
  // The second operator passes a const this pointer, and is useful
  // when operating on const references (e.g. any of our other
  // operators that receive const CVector3 & rhs as a parameter.)

  inline float32& operator[](int32 index);
  inline const float32& operator[](int32 index) const;

} vector4;

inline const vector4& vector4::clear() {
  x = 0;
  y = 0;
  z = 0;
  w = 0;

  return (*this);
}

inline const vector4& vector4::set(const vector4& rhs) { return (*this) = rhs; }

inline const vector4& vector4::set(float32 xj, float32 yj, float32 zj,
                                   float32 wj) {
  x = xj;
  y = yj;
  z = zj;
  w = wj;

  return (*this);
}

inline vector4 vector4::project(const vector4& rhs) const {
  // project rhs onto *this
  vector4 this_normal = normalize();

  float32 projected_length = rhs.dot(this_normal);

  return this_normal * projected_length;
}

inline vector4 vector4::clamp(float32 lower, float32 upper) const {
  vector4 output = (*this);

  if (output.x < lower) output.x = lower;
  if (output.x > upper) output.x = upper;

  if (output.y < lower) output.y = lower;
  if (output.y > upper) output.y = upper;

  if (output.z < lower) output.z = lower;
  if (output.z > upper) output.z = upper;

  if (output.w < lower) output.w = lower;
  if (output.w > upper) output.w = upper;

  return output;
}

inline vector4 vector4::normalize() const {
  vector4 output = *this;
  float32 l = length();

  if (l == 0.0) l = 1.0f;

  output.x = output.x / l;
  output.y = output.y / l;
  output.z = output.z / l;
  output.w = output.w / l;

  return output;
}

inline bool vector4::parallel(const vector4& rhs) const {
  // if vector 2 is some multiple of vector 1, they are parallel.
  float32 a = angle(rhs);

  if (compare_epsilon(a, 0.0f, BASE_EPSILON) ||
      compare_epsilon(a, BASE_PI, BASE_EPSILON)) {
    return true;
  }

  return false;
}

inline bool vector4::orthogonal(const vector4& rhs) const {
  return (dot(rhs) == 0);
}

inline float32 vector4::angle(const vector4& rhs) const {
  float32 product = dot(rhs);

  // cos(t) = v1 (dot) v2 / ||v1|| * ||v2||

  float32 len1 = length();
  float32 len2 = rhs.length();

  // If either vector has a zero length, the angle between them
  // will be invalid.

  if (0 == len1 || 0 == len2) {
    return 0.0f;
  }

  float32 v_angle = product / (len1 * len2);

  return (float32)acos(v_angle);
}

inline float32 vector4::dot(const vector4& rhs) const {
  return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
}

inline float32 vector4::distance(const vector4& rhs) const {
  vector4 vec_delta = rhs - (*this);

  return vec_delta.length();
}

inline float32 vector4::length() const { return sqrtf(dot(*this)); }

inline vector4 vector4::rotate(float32 rad, const vector3& axis) const {
  vector3 output(x, y, z);
  output = output.rotate(rad, axis);
  return output;
}

inline vector4 vector4::rotate_x(float32 rad) const {
  return rotate(rad, BASE_X_AXIS);
}

inline vector4 vector4::rotate_y(float32 rad) const {
  return rotate(rad, BASE_Y_AXIS);
}

inline vector4 vector4::rotate_z(float32 rad) const {
  return rotate(rad, BASE_Z_AXIS);
}

inline const vector4& vector4::operator=(const vector4& rhs) {
  x = rhs.x;
  y = rhs.y;
  z = rhs.z;
  w = rhs.w;

  return *this;
}

inline bool vector4::operator==(const vector4& rhs) const {
  return (x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w);
}

inline bool vector4::operator!=(const vector4& rhs) const {
  return !(*this == rhs);
}

inline vector4 vector4::operator-(const vector4& rhs) const {
  return vector4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
}

inline vector4 vector4::operator+(const vector4& rhs) const {
  return vector4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
}

inline const vector4& vector4::operator-=(const vector4& rhs) {
  return ((*this) = (*this) - rhs);
}

inline const vector4& vector4::operator+=(const vector4& rhs) {
  return ((*this) = (*this) + rhs);
}

inline vector4 vector4::operator*(const vector4& rhs) const {
  return vector4(x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w);
}

inline const vector4& vector4::operator*=(const vector4& rhs) {
  return ((*this) = (*this) * rhs);
}

inline vector4 vector4::operator*(float32 rhs) const {
  return vector4(x * rhs, y * rhs, z * rhs, w * rhs);
}

inline const vector4& vector4::operator*=(float32 rhs) {
  return ((*this) = (*this) * rhs);
}

inline vector4 vector4::operator/(const vector4& rhs) const {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs.x || 0 == rhs.y || 0 == rhs.z || 0 == rhs.w) {
      return (*this);
    }
  }

  return vector4(x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w);
}

inline const vector4& vector4::operator/=(const vector4& rhs) {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs.x || 0 == rhs.y || 0 == rhs.z || 0 == rhs.w) {
      return (*this);
    }
  }

  return ((*this) = (*this) / rhs);
}

inline vector4 vector4::operator/(float32 rhs) const {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs) {
      return (*this);
    }
  }

  return vector4(x / rhs, y / rhs, z / rhs, w / rhs);
}

inline const vector4& vector4::operator/=(float32 rhs) {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs) {
      return (*this);
    }
  }

  return ((*this) = (*this) / rhs);
}

inline float32& vector4::operator[](int32 index) {
  if (index > 3) return x;

  return v[index];
}

inline const float32& vector4::operator[](int32 index) const {
  if (index > 3) return x;

  return v[index];
}

}  // namespace base

#endif  // __VECTOR4_H__
