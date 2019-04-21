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

#include <string>
#include <vector>
#include "bvh.h"
#include "material.h"
#include "math/base.h"
#include "math/plane.h"
#include "math/vector3.h"

namespace base {

typedef struct ObjectCollision {
  // The portion along the ray that the collision occurred.
  float32 collision_param;
  // The coordinate where a collision occurred.
  vector3 collision_point;
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
  virtual const vector3 GetCenter() = 0;
  // Initializes the basic material properties of the object.
  void SetMaterial(::std::shared_ptr<Material> material);
  // Returns the default material of the object.
  Material *GetMaterial() { return material_.get(); }
  virtual bool Trace(const ray &trajectory, ObjectCollision *hit_info) = 0;

  Object();

 protected:
  ::std::shared_ptr<Material> material_;
};

class SphericalObject : public Object {
 public:
  SphericalObject(const vector3 &origin, float32 radius);
  const vector3 GetCenter() override { return origin_; }
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  float32 radius_;
  vector3 origin_;
};

class PlanarObject : public Object {
 public:
  PlanarObject(const plane &data);
  const vector3 GetCenter() override { return vector3(); }
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  plane plane_;
};

class DiscObject : public Object {
 public:
  DiscObject(const vector3 &origin, const vector3 &normal, float32 radius);
  const vector3 GetCenter() override { return origin_; }
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  plane plane_;
  float32 radius_;
  vector3 origin_;
};

class CubicObject : public Object {
 public:
  CubicObject(const vector3 &origin, float32 width, float32 height,
              float32 depth);
  const vector3 GetCenter() override { return cube_data_.query_center(); }
  void Rotate(const vector3 &axis, float32 angle);
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  cube cube_data_;
};

class MeshObject : public Object {
 public:
  MeshObject(const ::std::string &filename, bool invert_normals = false,
             const vector3 &translation = vector3(0, 0, 0), const vector3& scale = vector3(1, 1, 1));
  const vector3 GetCenter() override { return shape_tree.GetCenter(); }
  bool Trace(const ray &trajectory, ObjectCollision *hit_info) override;

 private:
  // The acceleration structure for the shape. Used to speed up traces.
  Bvh shape_tree;
  // The face list of the shape. References vertices in the parent mesh.
  ::std::vector<BvhFace> face_list;
  // The following lists are shared between all shapes.
  ::std::vector<vector3> vertices_;
  // The per-vertex normals for the mesh.
  ::std::vector<vector3> normals_;
  // The per-vertex texcoords for the mesh.
  ::std::vector<vector2> texcoords_;
  // If materials_ is empty, or any shape does not reference a material,
  // then the default Object-provided material will be used.
  ::std::vector<::std::unique_ptr<Material>> materials_;
};

}  // namespace base

#endif  // __OBJECT_H__