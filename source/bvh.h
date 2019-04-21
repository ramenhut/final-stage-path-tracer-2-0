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

#ifndef __BVH_H__
#define __BVH_H__

#include <memory>
#include <vector>
#include "math/base.h"
#include "math/scalar.h"
#include "math/trace.h"
#include "math/vector3.h"
#include "math/volume.h"

namespace base {

typedef struct BvhCollision {
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
  BvhCollision();
} BvhCollision;

typedef struct BvhFace {
  // Vertex indices for the face.
  uint32 vertex_indices[3];
  // Normal indices for the face.
  uint32 normal_indices[3];
  // Texcoord indices for the face.
  uint32 texcoord_indices[3];
  // Material index for the face.
  uint32 material;
} BvhFace;

class BvhNode {
 public:
  // Copy ctor that initializes the node.
  explicit BvhNode(BvhNode* parent_node);
  // Call by parent nodes to initialize the boundaries of the node.
  void SetBounds(const bounds& bb);
  // Returns the axis aligned bounds of the node.
  const bounds& GetBounds() const;
  // Called by parent nodes to insert faces into a node.
  void AddFace(uint32 index);
  // Checks if the node requires subdivision and, if necessary, allocates
  // child nodes and initiates recursive subdivision.
  void Subdivide();
  // Returns true if the node has no children. False otherwise.
  bool IsLeafNode() const;
  // Returns the index of the closest child node to the given point.
  uint8 ClosestChild(const vector3& point) const;
  // Traces a ray through the node and returns collision information.
  bool Trace(const ray& trajectory, BvhCollision* hit_info) const;

 private:
  friend class Bvh;
  BvhNode(::std::vector<vector3>* vertices, ::std::vector<BvhFace>* faces);
  // Optimized tracing of rays that begin outside of the node.
  bool TraceExternal(const collision node_hit, const ray& trajectory,
                     BvhCollision* hit_info) const;
  // Optimized tracing of rays that begin within the node.
  bool TraceInternal(const collision node_hit, const ray& trajectory,
                     BvhCollision* hit_info) const;

  bounds aabb_;
  uint32 depth_;
  BvhNode* parent_;
  bool is_leaf_node_;
  plane split_planes_[3];

  // Up to 8 child nodes attached to this node.
  ::std::unique_ptr<BvhNode> children_[8];
  // External list of vertices covered by this node.
  ::std::vector<vector3>* tree_vertices_;
  // External list of faces covered by this node.
  ::std::vector<BvhFace>* tree_faces_;
  // Face indices directly managed by this node.
  ::std::vector<uint32> face_indices_;
};

class Bvh {
  // Root node of the tree.
  ::std::unique_ptr<BvhNode> root_node_;
  // External list of vertices covered by this bvh.
  ::std::vector<vector3>* tree_vertices_;
  // External list of faces covered by this bvh.
  ::std::vector<BvhFace>* tree_faces_;

 public:
  const vector3 GetCenter();
  void BuildBvh(::std::vector<vector3>* vertices,
                ::std::vector<BvhFace>* faces);
  bool Trace(const ray& trajectory, BvhCollision* hit_info) const;
};

}  // namespace base

#endif  // __BVH_H__