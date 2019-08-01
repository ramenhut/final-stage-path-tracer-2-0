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

#ifndef __OBJECT_H__
#define __OBJECT_H__

#include <memory>
#include <string>
#include <vector>
#include "material.h"
#include "math/base.h"
#include "math/plane.h"
#include "math/trace.h"
#include "math/vector3.h"
#include "math/volume.h"

namespace base {

typedef struct ObjectCollision {
  // The portion along the ray that the collision occurred.
  float32 param;
  // The coordinate where a collision occurred.
  vector3 point;
  // The interpolated normal at the collision point.
  vector3 surface_normal;
  // The interpolated texcoords at the collision point.
  vector2 surface_texcoords;
  // The material at the surface that was struck.
  Material *surface_material;
  // True if the colliding ray originated inside the object.
  bool is_internal;
  ObjectCollision();
} ObjectCollision;

class Object {
 public:
  // Returns the center point of the object.
  virtual const vector3 GetCenter() const = 0;
  // Returns an axis aligned bounding box for the object's bounds.
  virtual const bounds GetBounds() const = 0;
  // Initializes the basic material properties of the object.
  void SetMaterial(::std::shared_ptr<Material> material);
  // Returns the default material of the object.
  Material *GetMaterial() { return material_.get(); }
  // Determines whether the ray intersects the object. Returns true if so,
  // false otherwise. If a collision is detected, hit_info will contain
  // information about the collision point.
  virtual bool Trace(const ray &trajectory, ObjectCollision *hit_info) = 0;

  Object();

 protected:
  ::std::shared_ptr<Material> material_;
};

class SphericalObject : public Object {
 public:
  SphericalObject(const vector3 &origin, float32 radius);
  const vector3 GetCenter() const override { return origin_; }
  const bounds GetBounds() const override { return aabb_; }
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  bounds aabb_;
  float32 radius_;
  vector3 origin_;
};

class PlanarObject : public Object {
 public:
  PlanarObject(const plane &data);
  const vector3 GetCenter() const override { return vector3(); }
  const bounds GetBounds() const override { return aabb_; }
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  bounds aabb_;
  plane plane_;
};

class DiscObject : public Object {
 public:
  DiscObject(const vector3 &origin, const vector3 &normal, float32 radius);
  const vector3 GetCenter() const override { return origin_; }
  const bounds GetBounds() const override { return aabb_; }
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  bounds aabb_;
  plane plane_;
  float32 radius_;
  vector3 origin_;
};

class CuboidObject : public Object {
 public:
  CuboidObject(const vector3 &origin, float32 width, float32 height,
               float32 depth);
  const vector3 GetCenter() const override { return cube_data_.query_center(); }
  const bounds GetBounds() const override { return cube_data_.query_bounds(); }
  void Rotate(const vector3 &axis, float32 angle);
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  cube cube_data_;
};

class QuadObject : public Object {
public:
    QuadObject(const vector3& origin, const vector3& normal, float32 width, float32 height);
    QuadObject(const vector3& position, const vector3& u, const vector3& v);
    const vector3 GetCenter() const override { return vector3(); }
    const bounds GetBounds() const override { return aabb_; }
    bool Trace(const ray& trajectory, ObjectCollision* hit_info) override;

private:
    bounds aabb_;
    plane plane_;
    float32 half_width_;
    float32 half_height_;
    vector3 origin_;
    vector3 bitangent_;
    vector3 tangent_;
};

}  // namespace base

#endif  // __OBJECT_H__