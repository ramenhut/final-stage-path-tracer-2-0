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

#ifndef __PLANE_H__
#define __PLANE_H__

#include "base.h"
#include "normal.h"
#include "scalar.h"
#include "vector3.h"
#include "vector4.h"

namespace base {

typedef vector4 plane;

inline vector3 project_plane(const plane& plane, const vector3& vector) {
  float32 d;
  vector3 n;
  float32 inv_denom;
  vector3 output;

  vector3 plane_norm(plane[0], plane[1], plane[2]);

  inv_denom = 1.0f / plane_norm.dot(vector);

  d = plane_norm.dot(vector) * inv_denom;

  n[0] = plane_norm[0] * inv_denom;
  n[1] = plane_norm[1] * inv_denom;
  n[2] = plane_norm[2] * inv_denom;

  output[0] = vector[0] - d * n[0];
  output[1] = vector[1] - d * n[1];
  output[2] = vector[2] - d * n[2];

  return output;
}

inline float32 plane_distance(const plane& plane, const vector3& point) {
  return (float32)plane[0] * point[0] + plane[1] * point[1] +
         plane[2] * point[2] + plane[3];
}

inline plane calculate_plane(const vector3& normal,
                             const vector3& pointOnPlane) {
  plane output;

  output[0] = normal[0];
  output[1] = normal[1];
  output[2] = normal[2];
  output[3] = -normal.dot(pointOnPlane);

  return output;
}

inline plane calculate_plane(const vector3& p1, const vector3& p2,
                             const vector3& p3, const vector3& pointOnPlane) {
  plane output;

  vector3 normal = calculate_normal(p1, p2, p3);

  output[0] = normal[0];
  output[1] = normal[1];
  output[2] = normal[2];
  output[3] = -normal.dot(pointOnPlane);

  return output;
}

inline plane calculate_plane(const vector3& p1, const vector3& p2,
                             const vector3& p3) {
  return calculate_plane(p1, p2, p3, p1);
}

inline plane normalize_plane(const plane& p) {
  plane output;
  float32 length = inv_sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);

  output[0] = p[0] * length;
  output[1] = p[1] * length;
  output[2] = p[2] * length;
  output[3] = p[3] * length;

  return output;
}

inline void calculate_tangent_basis(const vector3& p1, const vector3& p2,
                                    const vector3& p3, const vector2& t1,
                                    const vector2& t2, const vector2& t3,
                                    vector3* tangent, vector3* bitangent) {
  if (BASE_PARAM_CHECK) {
    if (!tangent || !bitangent) {
      return;
    }
  }

  vector3 abc0, abc1, abc2;

  vector3 vec1abc0 = vector3(p1.x - p2.x, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc0 = vector3(p1.x - p3.x, t1.x - t3.x, t1.y - t3.y);

  vector3 vec1abc1 = vector3(p1.y - p2.y, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc1 = vector3(p1.y - p3.y, t1.x - t3.x, t1.y - t3.y);

  vector3 vec1abc2 = vector3(p1.z - p2.z, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc2 = vector3(p1.z - p3.z, t1.x - t3.x, t1.y - t3.y);

  abc0 = vec1abc0.cross(vec2abc0);
  abc1 = vec1abc1.cross(vec2abc1);
  abc2 = vec1abc2.cross(vec2abc2);

  (*tangent) = vector3(-1.0 * abc0.y / abc0.x, -1.0 * abc1.y / abc1.x,
                       -1.0 * abc2.y / abc2.x)
                   .normalize();
  (*bitangent) = vector3(-1.0 * abc0.z / abc0.x, -1.0 * abc1.z / abc1.x,
                         -1.0 * abc2.z / abc2.x)
                     .normalize();
}

inline vector3 calculate_tangent(const vector3& p1, const vector3& p2,
                                 const vector3& p3, const vector2& t1,
                                 const vector2& t2, const vector2& t3) {
  vector3 abc0, abc1, abc2;

  vector3 vec1abc0 = vector3(p1.x - p2.x, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc0 = vector3(p1.x - p3.x, t1.x - t3.x, t1.y - t3.y);

  vector3 vec1abc1 = vector3(p1.y - p2.y, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc1 = vector3(p1.y - p3.y, t1.x - t3.x, t1.y - t3.y);

  vector3 vec1abc2 = vector3(p1.z - p2.z, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc2 = vector3(p1.z - p3.z, t1.x - t3.x, t1.y - t3.y);

  abc0 = vec1abc0.cross(vec2abc0);
  abc1 = vec1abc1.cross(vec2abc1);
  abc2 = vec1abc2.cross(vec2abc2);

  return vector3(-1.0 * abc0.y / abc0.x, -1.0 * abc1.y / abc1.x,
                 -1.0 * abc2.y / abc2.x)
      .normalize();
}

inline vector3 calculate_bitangent(const vector3& p1, const vector3& p2,
                                   const vector3& p3, const vector2& t1,
                                   const vector2& t2, const vector2& t3) {
  vector3 abc0, abc1, abc2;

  vector3 vec1abc0 = vector3(p1.x - p2.x, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc0 = vector3(p1.x - p3.x, t1.x - t3.x, t1.y - t3.y);

  vector3 vec1abc1 = vector3(p1.y - p2.y, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc1 = vector3(p1.y - p3.y, t1.x - t3.x, t1.y - t3.y);

  vector3 vec1abc2 = vector3(p1.z - p2.z, t1.x - t2.x, t1.y - t2.y);
  vector3 vec2abc2 = vector3(p1.z - p3.z, t1.x - t3.x, t1.y - t3.y);

  abc0 = vec1abc0.cross(vec2abc0);
  abc1 = vec1abc1.cross(vec2abc1);
  abc2 = vec1abc2.cross(vec2abc2);

  return vector3(-1.0 * abc0.z / abc0.x, -1.0 * abc1.z / abc1.x,
                 -1.0 * abc2.z / abc2.x)
      .normalize();
}

}  // namespace base

#endif  // __PLANE_H__
