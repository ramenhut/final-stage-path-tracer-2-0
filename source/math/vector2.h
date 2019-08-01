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

#ifndef __VECTOR2_H__
#define __VECTOR2_H__

#include "base.h"
#include "scalar.h"

namespace base {

typedef struct vector2 {
 public:
  union {
    struct {
      float32 x;
      float32 y;
    };

    float32 v[2];
  };

 public:
  vector2();
  vector2(const vector2& rhs);
  vector2(float32 xj, float32 yj);
  ~vector2();

  inline const vector2& clear();
  inline vector2 normalize() const;
  inline vector2 project(const vector2& rhs) const;
  inline vector2 reflect(const vector2& normal) const;
  inline vector2 Refract(const vector2& normal, float32 a, float32 b) const;
  inline vector2 clamp(float32 lower, float32 upper) const;

  inline const vector2& set(const vector2& rhs);
  inline const vector2& set(float32 xj, float32 yj);

  inline bool parallel(const vector2& rhs) const;
  inline bool orthogonal(const vector2& rhs) const;

  inline float32 angle(const vector2& rhs) const;
  inline float32 dot(const vector2& rhs) const;
  inline float32 distance(const vector2& rhs) const;
  inline float32 length() const;

  inline const vector2& operator=(const vector2& rhs);
  inline const vector2& operator-=(const vector2& rhs);
  inline const vector2& operator+=(const vector2& rhs);
  inline const vector2& operator*=(const vector2& rhs);
  inline const vector2& operator*=(float32 rhs);
  inline const vector2& operator/=(const vector2& rhs);
  inline const vector2& operator/=(float32 rhs);

  inline bool operator==(const vector2& rhs) const;
  inline bool operator!=(const vector2& rhs) const;
  inline vector2 operator-(const vector2& rhs) const;
  inline vector2 operator+(const vector2& rhs) const;
  inline vector2 operator*(const vector2& rhs) const;
  inline vector2 operator*(float32 rhs) const;
  inline vector2 operator/(const vector2& rhs) const;
  inline vector2 operator/(float32 rhs) const;

  // Here we define two variants of our bracket operator. The first
  // operator handles the lvalue case:
  //
  //     vector[0] = ...
  //
  // The second operator passes a const this pointer, and is useful
  // when operating on const references (e.g. any of our other
  // operators that receive const vector2 & rhs as a parameter.)

  inline float32& operator[](int32 index);
  inline const float32& operator[](int32 index) const;

} vector2;

inline const vector2& vector2::set(const vector2& rhs) { return (*this) = rhs; }

inline const vector2& vector2::set(float32 xj, float32 yj) {
  x = xj;
  y = yj;

  return (*this);
}

inline const vector2& vector2::clear() {
  x = 0;
  y = 0;

  return (*this);
}

inline vector2 vector2::clamp(float32 lower, float32 upper) const {
  vector2 output = (*this);

  if (output.x < lower) output.x = lower;
  if (output.x > upper) output.x = upper;

  if (output.y < lower) output.y = lower;
  if (output.y > upper) output.y = upper;

  return output;
}

inline vector2 vector2::normalize() const {
  vector2 output = *this;
  float32 l = length();

  if (l == 0.0) l = 1.0f;

  output.x = output.x / l;
  output.y = output.y / l;

  return output;
}

inline vector2 vector2::project(const vector2& rhs) const {
  vector2 this_normal = normalize();

  float32 projected_length = rhs.dot(this_normal);

  return this_normal * projected_length;
}

inline vector2 vector2::reflect(const vector2& normal) const {
  vector2 reflect = (*this) - (normal * (normal.dot(*this) * 2.0));

  return reflect.normalize();
}

inline vector2 vector2::Refract(const vector2& normal, float32 a,
                                float32 b) const {
  vector2 refract;
  float32 refract_coeff;
  float32 l_over_t = a / b;
  float32 n_dot_inc = normal.dot(*this);
  float32 sqrt_coeff =
      (1 - ((a * a) / (b * b))) * (1 - (n_dot_inc * n_dot_inc));

  if (sqrt_coeff < 0) sqrt_coeff = 1;

  refract_coeff = (l_over_t * n_dot_inc - sqrtf(sqrt_coeff));
  refract = (normal * refract_coeff) - (*this) * l_over_t;

  return refract.normalize();
}

inline bool vector2::parallel(const vector2& rhs) const {
  // if vector 2 is some multiple of vector 1, they are parallel.

  float32 a = angle(rhs);

  if (compare_epsilon(a, 0.0, BASE_EPSILON) ||
      compare_epsilon(a, BASE_PI, BASE_EPSILON)) {
    return true;
  }

  return false;
}

inline bool vector2::orthogonal(const vector2& rhs) const {
  return (dot(rhs) == 0);
}

inline float32 vector2::angle(const vector2& rhs) const {
  float32 product = dot(rhs);

  // cos(t) = v1 (dot) v2 / ||v1|| * ||v2||

  float32 v_angle;
  float32 len1 = length();
  float32 len2 = rhs.length();

  // If either vector has a zero length, the angle between them
  // will be invalid.

  if (0 == len1 || 0 == len2) {
    return 0.0f;
  }

  v_angle = product / (len1 * len2);

  return (float32)acos(v_angle);
}

inline float32 vector2::dot(const vector2& rhs) const {
  return x * rhs.x + y * rhs.y;
}

inline float32 vector2::distance(const vector2& rhs) const {
  vector2 vec_delta = rhs - (*this);

  return vec_delta.length();
}

inline float32 vector2::length() const { return sqrtf(dot(*this)); }

inline const vector2& vector2::operator=(const vector2& rhs) {
  x = rhs.x;
  y = rhs.y;

  return (*this);
}

inline const vector2& vector2::operator-=(const vector2& rhs) {
  return ((*this) = (*this) - rhs);
}

inline const vector2& vector2::operator+=(const vector2& rhs) {
  return ((*this) = (*this) + rhs);
}

inline const vector2& vector2::operator*=(const vector2& rhs) {
  return ((*this) = (*this) * rhs);
}

inline const vector2& vector2::operator*=(float32 rhs) {
  return ((*this) = (*this) * rhs);
}

inline const vector2& vector2::operator/=(const vector2& rhs) {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs.x || 0 == rhs.y) {
      return (*this);
    }
  }

  return ((*this) = (*this) / rhs);
}

inline const vector2& vector2::operator/=(float32 rhs) {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs) {
      return (*this);
    }
  }

  return ((*this) = (*this) / rhs);
}

inline bool vector2::operator==(const vector2& rhs) const {
  return (compare_epsilon(x, rhs.x) && compare_epsilon(y, rhs.y));
}

inline bool vector2::operator!=(const vector2& rhs) const {
  return !(*this == rhs);
}

inline vector2 vector2::operator-(const vector2& rhs) const {
  return vector2(x - rhs.x, y - rhs.y);
}

inline vector2 vector2::operator+(const vector2& rhs) const {
  return vector2(x + rhs.x, y + rhs.y);
}

inline vector2 vector2::operator*(const vector2& rhs) const {
  return vector2(x * rhs.x, y * rhs.y);
}

inline vector2 vector2::operator*(float32 rhs) const {
  return vector2(x * rhs, y * rhs);
}

inline vector2 vector2::operator/(const vector2& rhs) const {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs.x || 0 == rhs.y) {
      return (*this);
    }
  }

  return vector2(x / rhs.x, y / rhs.y);
}

inline vector2 vector2::operator/(float32 rhs) const {
  if (BASE_PARAM_CHECK) {
    if (0 == rhs) {
      return (*this);
    }
  }

  return vector2(x / rhs, y / rhs);
}

inline float32& vector2::operator[](int32 index) {
  if (index > 1) return x;

  return v[index];
}

inline const float32& vector2::operator[](int32 index) const {
  if (index > 1) return x;

  return v[index];
}

}  // namespace base

#endif  // __VECTOR2_H__
