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

#ifndef __VECTOR3_H__
#define __VECTOR3_H__

#include "base.h"
#include "scalar.h"
#include "vector2.h"

#include <stdio.h>

namespace base {

typedef struct vector3 {
 public:
  union {
    struct {
      float32 x;
      float32 y;
      float32 z;
    };

    struct {
      float32 r;
      float32 g;
      float32 b;
    };

    float32 v[3];
  };

 public:
  vector3();
  vector3(const vector2& rhs);
  vector3(const vector3& rhs);
  vector3(float32 xj, float32 yj, float32 zj);
  ~vector3();

  operator vector2();

  inline const vector3& clear();
  inline vector3 normalize() const;
  inline vector3 cross(const vector3& rhs) const;
  inline vector3 project(const vector3& rhs) const;
  inline float projected_length(const vector3& rhs) const;
  inline vector3 reflect(const vector3& normal,
                         float32 freflectivity = 1.0f) const;
  inline vector3 refract(const vector3& normal, float32 index) const;
  inline vector3 clamp(float32 lower, float32 upper) const;

  inline const vector3& set(const vector3& rhs);
  inline const vector3& set(float32 xj, float32 yj, float32 zj);

  inline vector3 rotate_x(float32 rad) const;
  inline vector3 rotate_y(float32 rad) const;
  inline vector3 rotate_z(float32 rad) const;
  inline vector3 rotate(float32 angle, const vector3& axis) const;

  inline bool parallel(const vector3& rhs) const;
  inline bool orthogonal(const vector3& rhs) const;

  inline float32 angle(const vector3& rhs) const;
  inline float32 angle_relative(const vector3& rhs, const vector3& ref) const;
  inline float32 dot(const vector3& rhs) const;
  inline float32 distance(const vector3& rhs) const;
  inline float32 length() const;

  const vector3& operator=(const vector2& rhs);
  inline const vector3& operator=(const vector3& rhs);
  inline const vector3& operator-=(const vector3& rhs);
  inline const vector3& operator+=(const vector3& rhs);
  inline const vector3& operator*=(const vector3& rhs);
  inline const vector3& operator*=(float32 rhs);
  inline const vector3& operator/=(const vector3& rhs);
  inline const vector3& operator/=(float32 rhs);

  inline bool operator==(const vector3& rhs) const;
  inline bool operator!=(const vector3& rhs) const;
  inline vector3 operator-(const vector3& rhs) const;
  inline vector3 operator+(const vector3& rhs) const;
  inline vector3 operator*(const vector3& rhs) const;
  inline vector3 operator*(float32 rhs) const;
  inline vector3 operator/(const vector3& rhs) const;
  inline vector3 operator/(float32 rhs) const;

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

} vector3;

const vector3 BASE_X_AXIS(1.0, 0.0, 0.0);
const vector3 BASE_Y_AXIS(0.0, 1.0, 0.0);
const vector3 BASE_Z_AXIS(0.0, 0.0, -1.0);

inline const vector3& vector3::set(const vector3& rhs) { return (*this) = rhs; }

inline const vector3& vector3::set(float32 xj, float32 yj, float32 zj) {
  x = xj;
  y = yj;
  z = zj;

  return (*this);
}

inline const vector3& vector3::clear() {
  x = 0;
  y = 0;
  z = 0;

  return (*this);
}

inline vector3 vector3::clamp(float32 lower, float32 upper) const {
  vector3 output = (*this);

  if (output.x < lower) output.x = lower;
  if (output.x > upper) output.x = upper;

  if (output.y < lower) output.y = lower;
  if (output.y > upper) output.y = upper;

  if (output.z < lower) output.z = lower;
  if (output.z > upper) output.z = upper;

  return output;
}

inline vector3 vector3::normalize() const {
  vector3 output = *this;
  float32 l = length();

  if (l == 0.0) l = 1.0f;

  output.x = output.x / l;
  output.y = output.y / l;
  output.z = output.z / l;

  return output;
}

inline vector3 vector3::cross(const vector3& rhs) const {
  vector3 output;

  output.x = (y * rhs.z) - (z * rhs.y);
  output.y = (z * rhs.x) - (x * rhs.z);
  output.z = (x * rhs.y) - (y * rhs.x);

  return output;
}

inline vector3 vector3::project(const vector3& rhs) const {
  vector3 this_normal = normalize();
  float32 projected_length = rhs.dot(this_normal);

  return this_normal * projected_length;
}

inline float vector3::projected_length(const vector3& rhs) const {
  return rhs.dot(*this);
}

inline vector3 vector3::reflect(const vector3& normal,
                                float32 freflectivity) const {
  return (*this) - (normal * (normal.dot(*this) * (freflectivity + 1.0f)));
}

inline vector3 vector3::refract(const vector3& normal, float32 index) const {
  float32 n_dot_v = -dot(normal);
  float32 sin2 = (index * index) * (1.0 - n_dot_v * n_dot_v);

  if (sin2 >= 1.0) return vector3();

  vector3 refraction =
      (*this) * index + normal * (index * n_dot_v - sqrtf(1.0 - sin2));
  return refraction.normalize();
}

inline bool vector3::parallel(const vector3& rhs) const {
  // if vector 2 is some multiple of vector 1, they are parallel.

  float32 a = angle(rhs);

  if (compare_epsilon(a, 0.0, BASE_EPSILON) ||
      compare_epsilon(a, BASE_PI, BASE_EPSILON)) {
    return true;
  }

  return false;
}

inline bool vector3::orthogonal(const vector3& rhs) const {
  return (dot(rhs) == 0);
}

inline float32 vector3::angle(const vector3& rhs) const {
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

inline float32 vector3::angle_relative(const vector3& rhs,
                                       const vector3& ref) const {
  float32 product = dot(rhs);
  float32 len1 = length();
  float32 len2 = rhs.length();

  // If either vector has a zero length, the angle between them
  // will be invalid.

  if (0 == len1 || 0 == len2) {
    return 0.0f;
  }

  float32 v_angle = product / (len1 * len2);

  // Here we check to make sure our two vectors are not extremely
  // close together. If they are, we simply return an angle of zero.

  if (compare_epsilon(v_angle, 1)) {
    return 0.0f;
  }

  float32 acosangle = acos(v_angle);

  // ATP: Our vectors are sufficiently different, so we continue
  //	    with our relative angle calculation.

  vector3 newAxis = cross(rhs);
  float32 angularSign = ref.dot(newAxis);

  if (angularSign < 0) {
    return BASE_PI + (BASE_PI - acosangle);
  }

  else
    return (float32)acosangle;
}

inline float32 vector3::dot(const vector3& rhs) const {
  return x * rhs.x + y * rhs.y + z * rhs.z;
}

inline float32 vector3::distance(const vector3& rhs) const {
  vector3 vec_delta = rhs - (*this);

  return vec_delta.length();
}

inline float32 vector3::length() const { return sqrtf(dot(*this)); }

inline vector3 vector3::rotate_x(float32 rad) const {
  return rotate(rad, BASE_X_AXIS);
}

inline vector3 vector3::rotate_y(float32 rad) const {
  return rotate(rad, BASE_Y_AXIS);
}

inline vector3 vector3::rotate_z(float32 rad) const {
  return rotate(rad, BASE_Z_AXIS);
}

inline vector3 vector3::rotate(float32 angle, const vector3& axis) const {
  float32 cosTheta = (float32)cos(angle);
  float32 sinTheta = (float32)sin(angle);
  vector3 outval;

  outval[0] = (cosTheta + (1 - cosTheta) * axis[0] * axis[0]) * x;
  outval[0] += ((1 - cosTheta) * axis[0] * axis[1] - axis[2] * sinTheta) * y;
  outval[0] += ((1 - cosTheta) * axis[0] * axis[2] + axis[1] * sinTheta) * z;

  outval[1] = ((1 - cosTheta) * axis[0] * axis[1] + axis[2] * sinTheta) * x;
  outval[1] += (cosTheta + (1 - cosTheta) * axis[1] * axis[1]) * y;
  outval[1] += ((1 - cosTheta) * axis[1] * axis[2] - axis[0] * sinTheta) * z;

  outval[2] = ((1 - cosTheta) * axis[0] * axis[2] - axis[1] * sinTheta) * x;
  outval[2] += ((1 - cosTheta) * axis[1] * axis[2] + axis[0] * sinTheta) * y;
  outval[2] += (cosTheta + (1 - cosTheta) * axis[2] * axis[2]) * z;

  return outval;
}

inline const vector3& vector3::operator=(const vector3& rhs) {
  x = rhs.x;
  y = rhs.y;
  z = rhs.z;

  return *this;
}

inline bool vector3::operator==(const vector3& rhs) const {
  return (compare_epsilon(x, rhs.x) && compare_epsilon(y, rhs.y) &&
          compare_epsilon(z, rhs.z));
}

inline bool vector3::operator!=(const vector3& rhs) const {
  return !(*this == rhs);
}

inline vector3 vector3::operator-(const vector3& rhs) const {
  return vector3(x - rhs.x, y - rhs.y, z - rhs.z);
}

inline vector3 vector3::operator+(const vector3& rhs) const {
  return vector3(x + rhs.x, y + rhs.y, z + rhs.z);
}

inline const vector3& vector3::operator-=(const vector3& rhs) {
  return ((*this) = (*this) - rhs);
}

inline const vector3& vector3::operator+=(const vector3& rhs) {
  return ((*this) = (*this) + rhs);
}

inline vector3 vector3::operator*(const vector3& rhs) const {
  return vector3(x * rhs.x, y * rhs.y, z * rhs.z);
}

inline const vector3& vector3::operator*=(const vector3& rhs) {
  return ((*this) = (*this) * rhs);
}

inline vector3 vector3::operator*(float32 rhs) const {
  return vector3(x * rhs, y * rhs, z * rhs);
}

inline const vector3& vector3::operator*=(float32 rhs) {
  return ((*this) = (*this) * rhs);
}

inline vector3 vector3::operator/(const vector3& rhs) const {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs.x || 0 == rhs.y || 0 == rhs.z) {
      return (*this);
    }
  }

  return vector3(x / rhs.x, y / rhs.y, z / rhs.z);
}

inline const vector3& vector3::operator/=(const vector3& rhs) {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs.x || 0 == rhs.y || 0 == rhs.z) {
      return (*this);
    }
  }

  return ((*this) = (*this) / rhs);
}

inline vector3 vector3::operator/(float32 rhs) const {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs) {
      return (*this);
    }
  }

  return vector3(x / rhs, y / rhs, z / rhs);
}

inline const vector3& vector3::operator/=(float32 rhs) {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs) {
      return (*this);
    }
  }

  return ((*this) = (*this) / rhs);
}

inline float32& vector3::operator[](int32 index) {
  if (index > 2) return x;

  return v[index];
}

inline const float32& vector3::operator[](int32 index) const {
  if (index > 2) return x;

  return v[index];
}

}  // namespace base

#endif  // __VECTOR3_H__
