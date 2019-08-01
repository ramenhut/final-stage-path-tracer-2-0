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

#ifndef __MESH_H__
#define __MESH_H__

#include <string>
#include <vector>
#include "bvh.h"
#include "material.h"
#include "math/base.h"
#include "math/plane.h"
#include "math/vector3.h"
#include "math/volume.h"

namespace base {

typedef struct MeshCollision {
  // The parametric value along a ray where a collision occurred. If no
  // collision occurred, this value will be < 0 or > 1.
  float32 param;
  // The coordinate where a collision occurred. Only valid if param is within
  // the valid range.
  vector3 point;
  // The normal at the collision point. Only valid if param is within the valid
  // range.
  vector3 normal;
  // The face that was struck.
  uint32 face_index;
  // The (x, y) barycentric coordinates within the face triangle.
  vector2 bary_coords;
  MeshCollision();
} MeshCollision;

typedef struct MeshFace {
  // Vertex indices for the face.
  uint32 vertex_indices[3];
  // Normal indices for the face.
  uint32 normal_indices[3];
  // Texcoord indices for the face.
  uint32 texcoord_indices[3];
  // The precomputed plane for vertex_indices.
  plane face_plane;
  // Material index for the face.
  uint32 material;
} MeshFace;

typedef struct MeshBvhDataSource {
  // External vertex list for the mesh.
  ::std::vector<vector3>* vertices;
  // External face list for the mesh.
  ::std::vector<MeshFace>* faces;
} MeshBvhDataSource;

class MeshBvhNode : public BaseBvhNode<MeshBvhDataSource, MeshCollision> {
 public:
  // Copy ctor that initializes the node.
  explicit MeshBvhNode(MeshBvhNode* parent_node);
  // Called by parent nodes to insert faces into a node.
  void AddFace(uint32 index);
  // Checks if the node requires subdivision and, if necessary, allocates
  // child nodes and initiates recursive subdivision.
  void Subdivide() override;
  // Traces a ray through the node and returns collision information.
  bool Trace(const ray& trajectory, MeshCollision* hit_info) const override;

 protected:
  friend class MeshBvh;
  MeshBvhNode(const MeshBvhDataSource& data_source);
  // Called to allocate a node at children_[index] with type according to a
  // derived class.
  virtual void AllocateChild(int32 index) override;
  // External list of vertices referenced by this node.
  ::std::vector<vector3>* tree_vertices_;
  // External list of faces referenced by this node.
  ::std::vector<MeshFace>* tree_faces_;
  // Face indices directly managed by this node.
  ::std::vector<uint32> face_indices_;
};

class MeshBvh {
  // Root node of the tree.
  ::std::unique_ptr<MeshBvhNode> root_node_;
  // External list of vertices referenced by this bvh.
  ::std::vector<vector3>* tree_vertices_;
  // External list of faces referenced by this bvh.
  ::std::vector<MeshFace>* tree_faces_;

 public:
  const vector3 GetCenter() const;
  void BuildBvh(::std::vector<vector3>* vertices,
                ::std::vector<MeshFace>* faces);
  bool Trace(const ray& trajectory, MeshCollision* hit_info) const;
};

class MeshObject : public Object {
 public:
  MeshObject(const ::std::string& filename, bool invert_normals = false,
             const vector3& translation = vector3(0, 0, 0),
             const vector3& scale = vector3(1, 1, 1),
             const vector4& rotation = vector4(0, 0, 0, 0));
  const vector3 GetCenter() const override { return shape_tree.GetCenter(); }
  const bounds GetBounds() const override { return aabb_; }
  bool Trace(const ray& trajectory, ObjectCollision* hit_info) override;

 private:
  bounds aabb_;
  // The acceleration structure for the shape. Used to speed up traces.
  MeshBvh shape_tree;
  // The face list of the shape. References vertices in the parent mesh.
  ::std::vector<MeshFace> face_list;
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

#endif  // __MESH_H__