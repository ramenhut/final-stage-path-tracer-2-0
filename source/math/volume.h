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

#ifndef __BOUNDS_H__
#define __BOUNDS_H__

#include "base.h"
#include "matrix4.h"
#include "plane.h"
#include "vector3.h"

namespace base {

class cube;

// A simple axis-aligned bounding box. These objects are used to compute object
// hierarchies and optimize scene traversal.

class bounds {
 public:
  bounds();
  const bounds& operator=(const bounds& rhs);
  const bounds& operator=(const cube& rhs);
  const bounds& operator+=(const vector3& rhs);
  const bounds& operator+=(const bounds& rhs);
  bounds operator+(const vector3& rhs) const;
  bounds operator+(const bounds& rhs) const;
  bool operator==(const bounds& rhs) const;
  bool operator!=(const bounds& rhs) const;

  // Note that there is a distinction between the center of
  // a bounding box and the base, as described below:
  //
  //     o----------------------o max
  //     |                      |
  //     |                      |
  //     |                      |
  //     |        center        |
  //     |                      |
  //     |                      |
  //     |                      |
  // min o-------- base --------o

  void clear();
  void set_center(const vector3& new_center);
  void set_base(const vector3& new_base);
  void rotate(const vector3& axis, float32 angle);
  void scale(const vector3& s);
  void translate(const vector3& trans);
  bounds transform(const matrix4& xform) const;

  void set_min(const vector3& new_min);
  void set_max(const vector3& new_max);

  vector3 query_center() const;
  // Volume must be > 0 for valid and initialized bounds.
  float query_volume() const;
  // Returns the number of points that contributed to the bounds.
  uint32 query_vector_count() const;

 public:
  uint32 vector_count;
  vector3 bounds_min;
  vector3 bounds_max;
};

// A simple non-axis-aligned bounding cube. Cubes are useful when you
// need to rotate an axis-aligned bounding volume. The only way to create
// a cube is from a bounds -- we do not currently support generic cubes
// in the engine.

class cube {
 public:
  cube();
  cube(const bounds& rhs);

  const cube& operator=(const cube& rhs);
  const cube& operator=(const bounds& rhs);
  bool operator==(const cube& rhs) const;
  bool operator!=(const cube& rhs) const;

  float32 angle_x() const;
  float32 angle_y() const;
  float32 angle_z() const;

  vector3 query_min() const;
  vector3 query_max() const;
  vector3 query_center() const;
  bounds query_bounds() const;
  const plane& query_plane(uint8 index) const;

  void set_center(const vector3& center);
  void rotate(const vector3& axis, float32 angle);
  void scale(const vector3& s);
  void translate(const vector3& t);
  cube transform(const matrix4& xform) const;

 private:
  // Computes 6 face planes from the cube vertices.
  void compute_planes();
  friend class bounds;
  bounds aabb_;
  vector3 vertices[8];
  plane face_planes[6];
};

}  // namespace base

#endif  // __BOUNDS_H__
