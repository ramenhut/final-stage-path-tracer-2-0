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

#ifndef __TRACE_H__
#define __TRACE_H__

#include "base.h"
#include "vector3.h"

namespace base {

typedef struct ray {
  // The starting point of the ray.
  vector3 start;
  // The endpoint of the ray.
  vector3 stop;
  // The un-normalized ray direction.
  vector3 dir;
  ray(const vector3& init_start, const vector3& init_stop);
  // Computes the length as length(end - start).
  float length() const;
  // Computes the direction of the ray as normalize(end - start).
  vector3 direction() const;
  // Checks if the ray's length is within epsilon of zero.
  bool has_zero_length() const;
} ray;

typedef struct collision {
  // The parametric value along a ray where a collision occurred. If no
  // collision occurred, this value will be < 0 or > 1.
  float param;
  // The coordinate where a collision occurred. Only valid if param is within
  // the valid range.
  vector3 point;
  // The normal at the collision point. Only valid if param is within the valid
  // range.
  vector3 normal;

  collision();
  bool operator<(const collision& rhs) const;
  bool operator>(const collision& rhs) const;
  bool operator<=(const collision& rhs) const;
  bool operator>=(const collision& rhs) const;
} collision;

}  // namespace base

#endif  // __VN_TRACE_H__
