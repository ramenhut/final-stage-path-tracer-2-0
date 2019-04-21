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

#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "base.h"
#include "matrix3.h"
#include "vector3.h"
#include "vector4.h"

namespace base {

typedef struct quat {
  union {
    struct {
      float32 w;
      float32 x;
      float32 y;
      float32 z;
    };

    float32 q[4];
  };

 public:
  quat();
  quat(const vector3& rhs);
  quat(const quat& rhs);
  quat(float32 wj, float32 xj, float32 yj, float32 zj);
  ~quat();

  inline const quat& clear();
  inline quat normalize() const;
  inline float32 length() const;

  inline const quat& set(const quat& rhs);
  inline const quat& set(float32 wj, float32 xj, float32 yj, float32 zj);
  inline quat clamp(float32 lower, float32 upper) const;
  inline quat inverse() const;
  inline quat Conjugate() const;
  inline float32 dot(const quat& rhs) const;
  inline float32 angle(const quat& rhs) const;
  inline vector3 axis(const quat& rhs) const;

  inline vector3 rotate(const vector3& rhs) const;
  inline quat rotation(const vector3& from, const vector3& to) const;
  inline quat rotation(const quat& to, const float32 frac) const;

  inline const quat& from_euler(const vector3& rhs);
  inline vector4 to_axis_angle() const;  // returned as <x, y, z, angle>
  inline const quat& from_axis_angle(const vector3& axis, float32 angle);
  inline const quat& from_axis_angle(const vector4& rhs);
  matrix3 to_matrix() const;

  inline const quat& operator=(const quat& rhs);
  inline const quat& operator*=(const quat& rhs);
  inline bool operator==(const quat& rhs) const;
  inline bool operator!=(const quat& rhs) const;
  inline quat operator+(const quat& rhs) const;
  inline quat operator*(const quat& rhs) const;
  inline quat operator*(
      const float32& rhs) const;  // simple component multiplication
  inline vector3 operator*(
      const vector3& rhs) const;  // rotates a vector by this quaternion

  // Here we define two variants of our bracket operator. The first
  // operator handles the lvalue case:
  //
  //     quat[0] = ...
  //
  // The second operator passes a const this pointer, and is useful
  // when operating on const references (e.g. any of our other
  // operators that receive const CVector3 & rhs as a parameter.)

  inline float32& operator[](int32 index);
  inline const float32& operator[](int32 index) const;

} quaternion;

inline const quat& quat::clear() {
  w = x = y = z = 0;

  return (*this);
}

inline float32 quat::length() const {
  quat temp = *this;
  return sqrt(dot(temp));
}

inline quat quat::normalize() const {
  quat q = (*this);

  float32 len = q.length();

  q.x /= len;
  q.y /= len;
  q.z /= len;
  q.w /= len;

  return q;
}

inline const quat& quat::set(const quat& rhs) { return (*this) = rhs; }

inline const quat& quat::set(float32 wj, float32 xj, float32 yj, float32 zj) {
  x = xj;
  y = yj;
  z = zj;
  w = wj;

  return (*this);
}

inline quat quat::clamp(float32 lower, float32 upper) const {
  quat c = (*this);

  if (c.x < lower) c.x = lower;
  if (c.x > upper) c.x = upper;

  if (c.y < lower) c.y = lower;
  if (c.y > upper) c.y = upper;

  if (c.z < lower) c.z = lower;
  if (c.z > upper) c.z = upper;

  if (c.w < lower) c.w = lower;
  if (c.w > upper) c.w = upper;

  return c;
}

inline quat quat::inverse() const {
  return (*this).Conjugate() * (1.0f / (length() * length()));
}

inline quat quat::Conjugate() const {
  quat q = (*this);

  q.x *= -1.0f;
  q.y *= -1.0f;
  q.z *= -1.0f;

  return q;
}

inline float32 quat::dot(const quat& rhs) const {
  return (x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w);
}

inline quat quat::rotation(const quat& to, float32 frac) const {
  // This serves as the basis for our spherical linear interpolation routines.

  quat q = (*this) * (1 - frac) + to * frac;

  return q.normalize();
}

inline float32 quat::angle(const quat& rhs) const {
  quat rotateBetween = inverse() * rhs;

  return rotateBetween.to_axis_angle().w;
}

inline vector3 quat::axis(const quat& rhs) const {
  quat r = inverse() * rhs;
  vector4 v = r.to_axis_angle();

  return vector3(v.x, v.y, v.z);
}

inline quat quat::rotation(const vector3& from, const vector3& to) const {
  quat q;
  vector3 axis = from.cross(to);
  float32 angle = from.angle(to);

  return ((quat)q.from_axis_angle(axis, angle));
}

inline vector4 quat::to_axis_angle() const {
  vector4 v;
  float32 len = length();

  v.x = x / len;
  v.y = y / len;
  v.z = z / len;
  v.w = acos(w) * 2.0f;

  return v;
}

inline const quat& quat::from_euler(const vector3& rhs) {
  float32 p = rhs.x / 2.0;
  float32 y = rhs.y / 2.0;
  float32 r = rhs.z / 2.0;

  float32 sinp = sin(p);
  float32 siny = sin(y);
  float32 sinr = sin(r);
  float32 cosp = cos(p);
  float32 cosy = cos(y);
  float32 cosr = cos(r);

  x = sinr * cosp * cosy - cosr * sinp * siny;
  y = cosr * sinp * cosy + sinr * cosp * siny;
  z = cosr * cosp * siny - sinr * sinp * cosy;
  w = cosr * cosp * cosy + sinr * sinp * siny;

  return ((*this) = normalize());
}

inline const quat& quat::from_axis_angle(const vector3& axis, float32 angle) {
  vector3 v = axis.normalize();
  float32 s = sin(angle * 0.5f);
  float32 c = cos(angle * 0.5f);

  x = (v.x * s);
  y = (v.y * s);
  z = (v.z * s);
  w = c;

  return (*this);
}

inline const quat& quat::from_axis_angle(const vector4& rhs) {
  vector3 axis(rhs.x, rhs.y, rhs.z);

  return from_axis_angle(axis, rhs.w);
}

inline vector3 quat::rotate(const vector3& rhs) const { return (*this) * rhs; }

inline quat quat::operator+(const quat& rhs) const {
  return quat(w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z);
}

inline quat quat::operator*(const float32& rhs) const {
  return quat(w * rhs, x * rhs, y * rhs, z * rhs);
}

inline vector3 quat::operator*(const vector3& rhs) const {
  // v = q * v * q' (conjugate)

  quat v(rhs.normalize());
  quat q = (*this) * v * Conjugate();

  return (vector3(q.x, q.y, q.z));
}

inline const quat& quat::operator=(const quat& rhs) {
  w = rhs.w;
  x = rhs.x;
  y = rhs.y;
  z = rhs.z;

  return (*this);
}

inline const quat& quat::operator*=(const quat& rhs) {
  return (*this) = (*this) * rhs;
}

inline bool quat::operator==(const quat& rhs) const {
  return (x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.z);
}

inline bool quat::operator!=(const quat& rhs) const { return !(*this == rhs); }

inline quat quat::operator*(const quat& rhs) const {
  return quat(w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
              w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
              w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
              w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
}

inline float32& quat::operator[](int32 index) {
  if (index > 3) return x;

  return q[index];
}

inline const float32& quat::operator[](int32 index) const {
  if (index > 3) return x;

  return q[index];
}

}  // namespace base

#endif  // __QUATERNION_H__
