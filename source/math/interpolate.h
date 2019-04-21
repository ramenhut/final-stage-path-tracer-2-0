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

#ifndef __INTERPOLATE_H__
#define __INTERPOLATE_H__

#include "base.h"
#include "intersect.h"
#include "vector2.h"
#include "vector3.h"
#include "vector4.h"

namespace base {

inline float32 lerp(float32 a, float32 b, float32 frac) {
  return a + (b - a) * frac;
}

inline vector2 lerp(const vector2& a, const vector2& b, float32 frac) {
  return a + (b - a) * frac;
}

inline vector3 lerp(const vector3& a, const vector3& b, float32 frac) {
  return a + (b - a) * frac;
}

inline vector4 lerp(const vector4& a, const vector4& b, float32 frac) {
  return a + (b - a) * frac;
}

inline vector2 slerp(const vector2& a, const vector2& b, float32 frac) {
  vector2 out;
  vector2 vec = b - a;
  float32 ang = a.angle(vec);
  float32 t1 = (sin(frac - 1) * ang) / ((1 - frac) * ang);
  float32 t2 = (sin(frac * ang) / (frac * ang));

  t1 *= (1 - frac);
  t2 *= frac;

  t1 /= sin(ang) / ang;
  t2 /= sin(ang) / ang;

  out[0] = t1 * a[0] + t2 * vec[0];
  out[1] = t1 * a[1] + t2 * vec[1];

  return out;
}

inline vector3 slerp(const vector3& a, const vector3& b, float32 frac) {
  vector3 out;
  vector3 vec = b - a;
  float32 ang = a.angle(vec);
  float32 t1 = (sin(frac - 1) * ang) / ((1 - frac) * ang);
  float32 t2 = (sin(frac * ang) / (frac * ang));

  t1 *= (1 - frac);
  t2 *= frac;

  t1 /= sin(ang) / ang;
  t2 /= sin(ang) / ang;

  out[0] = t1 * a[0] + t2 * vec[0];
  out[1] = t1 * a[1] + t2 * vec[1];
  out[2] = t1 * a[2] + t2 * vec[2];

  return out;
}

inline vector4 slerp(const vector4& a, const vector4& b, float32 frac) {
  vector4 out;
  vector4 vec = b - a;
  float32 ang = a.angle(vec);
  float32 t1 = (sin(frac - 1) * ang) / ((1 - frac) * ang);
  float32 t2 = (sin(frac * ang) / (frac * ang));

  t1 *= (1 - frac);
  t2 *= frac;

  t1 /= sin(ang) / ang;
  t2 /= sin(ang) / ang;

  out[0] = t1 * a[0] + t2 * vec[0];
  out[1] = t1 * a[1] + t2 * vec[1];
  out[2] = t1 * a[2] + t2 * vec[2];
  out[3] = t1 * a[3] + t2 * vec[3];

  return out;
}

inline vector3 color_lerp(const vector3& p0, const vector3& c0,
                          const vector3& p1, const vector3& c1,
                          const vector3& p2, const vector3& c2,
                          const vector3& pos) {
  // Our parameters define a triangle with three points and three colors.
  // The 'pos' parameter is expected to be a point within the triangle.
  //
  // If it is not, then we return black. Otherwise, we perform a standard
  // interpolation of the coordinates to derive the interpolated color value.
  vector2 bary_coords;
  if (!point_in_triangle(p0, p1, p2, pos, &bary_coords)) {
    return vector3(0, 0, 0);
  }

  vector3 result;
  if ((bary_coords.x >= 0) && (bary_coords.y >= 0) && (bary_coords.x + bary_coords.y <= 1)) {
    vector3 vc0 = (c1) - (c0);
    vector3 vc1 = (c2) - (c0);
    vector3 result_a = (c0 + vc0 * bary_coords.x);
    vector3 result_b = (result_a + vc1 * bary_coords.y);
    result = result_b;
  }

  return result;
}

inline float32 ease_in(float32 a, float32 b, float32 frac) {
  float32 dist = b - a;
  return dist * frac * frac + a;
}

inline vector2 ease_in(const vector2& a, const vector2& b, float32 frac) {
  vector2 dist = b - a;
  return dist * frac * frac + a;
}

inline vector3 ease_in(const vector3& a, const vector3& b, float32 frac) {
  vector3 dist = b - a;
  return dist * frac * frac + a;
}

inline vector4 ease_in(const vector4& a, const vector4& b, float32 frac) {
  vector4 dist = b - a;
  return dist * frac * frac + a;
}

inline float32 ease_over(float32 a, float32 b, float32 frac) {
  float32 dist = b - a;
  static const float32 fOver = 1.25f;
  static const float32 fTrav = fOver * BASE_PI;
  static const float32 fSent = 1.0f / (1.0 - cos(fTrav));
  return a + dist * fSent * (1.0 - cos(fTrav * frac));
}

inline vector2 ease_over(const vector2& a, const vector2& b, float32 frac) {
  vector2 dist = b - a;
  static const float32 fOver = 1.25f;
  static const float32 fTrav = fOver * BASE_PI;
  static const float32 fSent = 1.0f / (1.0 - cos(fTrav));
  return a + dist * fSent * (1.0 - cos(fTrav * frac));
}

inline vector3 ease_over(const vector3& a, const vector3& b, float32 frac) {
  vector3 dist = b - a;
  static const float32 fOver = 1.25f;
  static const float32 fTrav = fOver * BASE_PI;
  static const float32 fSent = 1.0f / (1.0 - cos(fTrav));
  return a + dist * fSent * (1.0 - cos(fTrav * frac));
}

inline vector4 ease_over(const vector4& a, const vector4& b, float32 frac) {
  vector4 dist = b - a;
  static const float32 fOver = 1.25f;
  static const float32 fTrav = fOver * BASE_PI;
  static const float32 fSent = 1.0f / (1.0 - cos(fTrav));
  return a + dist * fSent * (1.0 - cos(fTrav * frac));
}

}  // namespace base

#endif  // __INTERPOLATE_H__
