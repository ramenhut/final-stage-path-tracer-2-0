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

#ifndef __NORMAL_H__
#define __NORMAL_H__

#include <vector>
#include "base.h"
#include "vector3.h"

namespace base {

class normal_sphere {
 public:
  normal_sphere();
  // Initializes the normal sphere with count normals.
  void initialize(int32 count);
  // Returns a generic, unconstrained random normal from the unit sphere.
  vector3 random_normal();
  // Returns a random reflection normal within the solid angle envelope.
  vector3 random_reflection(const vector3& incident, const vector3& normal,
                            float32 solidangle);
  // Returns a random refraction normal within the solid angle envelope.
  vector3 random_refraction(const vector3& incident, const vector3& normal,
                            float32 solidangle, float32 n_l, float32 n_t);

 private:
  // The normal list is allocated and populated during setup,
  // and should never be reallocated during the frame. Access
  // into the normal list should be direct.
  std::vector<vector3> normal_list;
};

// Generic normal manipulation:
//   Note that the reflection and refraction interfaces here assume a solid
//   angle of zero. For non-zero solid angle envelopes, use the random normal
//   generator above.

inline vector3 calculate_normal(const vector3& a, const vector3& b,
                                const vector3& c) {
  vector3 tmpvec1;
  vector3 tmpvec2;
  vector3 crossed;

  tmpvec1 = b - a;
  tmpvec2 = c - a;

  crossed = tmpvec1.cross(tmpvec2);

  return crossed.normalize();
}

inline vector3 calculate_reflection(const vector3& incident,
                                    const vector3& normal,
                                    float32 reflectivity) {
  return incident.reflect(normal, reflectivity);
}

inline vector3 calculate_refraction(const vector3& incident,
                                    const vector3& normal, float32 n_l,
                                    float32 n_t) {
  return incident.refract(normal, n_l / n_t);
}

inline vector3 calculate_planar_projection(const vector3& vector_to_project,
                                           const vector3& normal) {
  return vector_to_project - normal * vector_to_project.dot(normal);
}

inline vector3 calculate_descent(const vector3& normal) {
  const vector3 gravity_vector(0, -1, 0);

  return calculate_planar_projection(gravity_vector, normal);
}

inline vector3 calculate_ascent(const vector3& normal) {
  const vector3 inv_gravity_vector(0, 1, 0);

  return calculate_planar_projection(inv_gravity_vector, normal);
}

}  // namespace base

#endif  // __NORMAL_H__
