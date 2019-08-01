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
#include "math/intersect.h"
#include "math/scalar.h"
#include "math/trace.h"
#include "math/vector3.h"
#include "math/volume.h"
#include "object.h"

namespace base {

// A simple class that provides common bvh node functionality without defining
// the bounded content type or recursive operations. Clients are expected to 
// derive this class to fit the needs of their data and operations.
template <class DataSource, class CollisionInfo>
class BaseBvhNode {
 public:
  // Call by parent nodes to initialize the boundaries of the node.
  void SetBounds(const bounds& bb);
  // Returns the axis aligned bounds of the node.
  const bounds& GetBounds() const;
  // Checks if the node requires subdivision and, if necessary, allocates
  // child nodes and initiates recursive subdivision.
  virtual void Subdivide() = 0;
  // Traces a ray through the node and returns collision information.
  virtual bool Trace(const ray& trajectory, CollisionInfo* hit_info) const = 0;

 protected:
  // Protected ctor to ensure that this class is never directly instantiated.
  BaseBvhNode();
  // Called to allocate a node at children_[index] with type according to a 
  // derived class.
  virtual void AllocateChild(int32 index) = 0;
  // Helper routine that allocates children and configures their bounds for
  // subdivision.
  void ConfigureChildren();
  // Returns true if the node has no children. False otherwise.
  bool IsLeafNode() const;
  // Returns the index of the closest child node to the given point.
  uint8 ClosestChild(const vector3& point) const;
  // Optimized ray-node collision detection. Returns true if a collision is
  // found, false otherwise.
  bool TraceInternal(const collision node_hit, const ray& trajectory,
      CollisionInfo* hit_info) const;

  bounds aabb_;
  uint32 depth_;
  bool is_leaf_node_;
  plane split_planes_[3];
  // Link to the parent node in the bvh hierarchy.
  BaseBvhNode<DataSource, CollisionInfo>* parent_;
  // Up to 8 child nodes attached to this node.
  ::std::unique_ptr<BaseBvhNode<DataSource, CollisionInfo>> children_[8];
};

template <class DataSource, class CollisionInfo>
BaseBvhNode<DataSource, CollisionInfo>::BaseBvhNode() {
  depth_ = 0;
  parent_ = nullptr;
  is_leaf_node_ = true;
}

template <class DataSource, class CollisionInfo>
void BaseBvhNode<DataSource, CollisionInfo>::SetBounds(const bounds& bb) {
  aabb_ = bb;
}

template <class DataSource, class CollisionInfo>
const bounds& BaseBvhNode<DataSource, CollisionInfo>::GetBounds() const {
  return aabb_;
}

template <class DataSource, class CollisionInfo>
void BaseBvhNode<DataSource, CollisionInfo>::ConfigureChildren() {
  vector3 min = aabb_.bounds_min;
  vector3 max = aabb_.bounds_max;
  vector3 center = aabb_.query_center();
  vector3 half_x = vector3(center.x, min.y, min.z) - min;
  vector3 half_y = vector3(min.x, center.y, min.z) - min;
  vector3 half_z = vector3(min.x, min.y, center.z) - min;
  vector3 node_span = half_x + half_y + half_z;

  split_planes_[0] = calculate_plane(vector3(1, 0, 0), center);
  split_planes_[1] = calculate_plane(vector3(0, 1, 0), center);
  split_planes_[2] = calculate_plane(vector3(0, 0, 1), center);

  for (uint8 i = 0; i < 8; i++) {
    AllocateChild(i);

    vector3 node_min =
        min + half_x * (i % 2) + half_y * (i >> 2) + half_z * ((i % 4) >> 1);
    vector3 node_max = node_min + node_span;
    bounds node_bounds;
    node_bounds += node_min;
    node_bounds += node_max;

    children_[i]->SetBounds(node_bounds);
  }

  // This node now has children, so it can no longer be considered a leaf.
  is_leaf_node_ = false;
}

template <class DataSource, class CollisionInfo>
bool BaseBvhNode<DataSource, CollisionInfo>::IsLeafNode() const {
  return is_leaf_node_;
}

template <class DataSource, class CollisionInfo>
uint8 BaseBvhNode<DataSource, CollisionInfo>::ClosestChild(
    const vector3& point) const {
  vector3 trace_dir = point - aabb_.query_center();
  uint32 x_test = trace_dir.x >= 0.0f;
  uint32 y_test = trace_dir.y >= 0.0f;
  uint32 z_test = trace_dir.z >= 0.0f;
  return x_test | (z_test << 1) | (y_test << 2);
}

template <class DataSource, class CollisionInfo>
bool BaseBvhNode<DataSource, CollisionInfo>::TraceInternal(
    const collision node_hit, const ray& trajectory,
    CollisionInfo* hit_info) const {
  bool trace_result = false;
  bool x_hit, y_hit, z_hit;
  uint8 closest_node = -1;
  ray internal_trajectory = trajectory;
  collision x_plane_info, y_plane_info, z_plane_info;

  if (!point_in_bounds(aabb_, trajectory.start)) {
    internal_trajectory.start = node_hit.point;
    closest_node = ClosestChild(node_hit.point);
    x_hit = ray_intersect_plane(split_planes_[0], internal_trajectory,
                                &x_plane_info);
    y_hit = ray_intersect_plane(split_planes_[1], internal_trajectory,
                                &y_plane_info);
    z_hit = ray_intersect_plane(split_planes_[2], internal_trajectory,
                                &z_plane_info);
  } else {
    closest_node = ClosestChild(trajectory.start);
    x_hit = ray_intersect_plane(split_planes_[0], trajectory, &x_plane_info);
    y_hit = ray_intersect_plane(split_planes_[1], trajectory, &y_plane_info);
    z_hit = ray_intersect_plane(split_planes_[2], trajectory, &z_plane_info);

    // If we didn't hit any of the planes then we're tracing out of the node
    // and we only need to check our current node for collisions.
    if (!x_hit && !y_hit && !z_hit) {
      if (children_[closest_node].get()) {
        return children_[closest_node]->Trace(trajectory, hit_info);
      }
      return false;
    }
  }

  for (uint32 i = 0; i < 4; i++) {
    if (children_[closest_node].get()) {
      if (children_[closest_node]->Trace(trajectory, hit_info)) {
        trace_result = true;
        if (point_in_bounds(children_[closest_node]->aabb_, hit_info->point)) {
          break;
        }
      }
    }

    if (x_hit && x_plane_info.param < y_plane_info.param &&
        x_plane_info.param < z_plane_info.param) {
      // x is the closest plane. we search the other x node.
      closest_node ^= 0x1;
      x_hit = false;
      internal_trajectory.start = x_plane_info.point;
      x_plane_info.param = BASE_INFINITY;
    } else if (y_hit && y_plane_info.param < z_plane_info.param &&
               y_plane_info.param < x_plane_info.param) {
      // y is the closest plane. we search the adjacent y node.
      closest_node ^= 0x4;
      y_hit = false;
      internal_trajectory.start = y_plane_info.point;
      y_plane_info.param = BASE_INFINITY;
    } else if (z_hit && z_plane_info.param < y_plane_info.param &&
               z_plane_info.param < x_plane_info.param) {
      // z is the closest plane. we search the adjacent z node.
      closest_node ^= 0x2;
      z_hit = false;
      internal_trajectory.start = z_plane_info.point;
      z_plane_info.param = BASE_INFINITY;
    } else {
      break;
    }

    // if the new ray start isn't in our node's bounding box, return false
    if (!point_in_bounds(aabb_, internal_trajectory.start)) {
      break;
    }
  }

  return trace_result;
}

}  // namespace base

#endif  // __BVH_H__