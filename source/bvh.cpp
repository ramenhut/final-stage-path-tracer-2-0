
#include "bvh.h"
#include "math/intersect.h"
#include "math/random.h"

namespace base {

const uint32 kMaxFaceCountPerNode = 16;
const uint32 kMaxSubdivisionDepth = 32;
const float32 kMinSubdivisionVolume = 0.001f;

BvhCollision::BvhCollision() : param(2.0), face_index(-1) {}

BvhNode::BvhNode(::std::vector<vector3>* vertices,
                 ::std::vector<BvhFace>* faces) {
  depth_ = 0;
  parent_ = nullptr;
  is_leaf_node_ = true;
  tree_vertices_ = vertices;
  tree_faces_ = faces;
}

BvhNode::BvhNode(BvhNode* parent_node) {
  depth_ = parent_node->depth_ + 1;
  parent_ = parent_node;
  is_leaf_node_ = true;
  tree_vertices_ = parent_node->tree_vertices_;
  tree_faces_ = parent_node->tree_faces_;
}

void BvhNode::SetBounds(const bounds& bb) { aabb_ = bb; }

void BvhNode::AddFace(uint32 index) { face_indices_.push_back(index); }

bool BvhNode::IsLeafNode() const {
  return is_leaf_node_;
  ;
}

const bounds& BvhNode::GetBounds() const { return aabb_; }

void BvhNode::Subdivide() {
  if (depth_ >= kMaxSubdivisionDepth) {
    return;
  }

  if (face_indices_.size() > kMaxFaceCountPerNode &&
      aabb_.query_volume() > kMinSubdivisionVolume) {
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
      children_[i].reset(new BvhNode(this));

      vector3 node_min =
          min + half_x * (i % 2) + half_y * (i >> 2) + half_z * ((i % 4) >> 1);
      vector3 node_max = node_min + node_span;
      bounds node_bounds;
      node_bounds += node_min;
      node_bounds += node_max;

      children_[i]->SetBounds(node_bounds);
    }

    // Compare all of the parent node's faces against the newly
    // created child. Add faces to the child that intersect it.
    for (uint32 j = 0; j < face_indices_.size(); j++) {
      uint32 face_index = face_indices_.at(j);
      const BvhFace& face = tree_faces_->at(face_index);
      const vector3& v0 = tree_vertices_->at(face.vertex_indices[0]);
      const vector3& v1 = tree_vertices_->at(face.vertex_indices[1]);
      const vector3& v2 = tree_vertices_->at(face.vertex_indices[2]);
      for (uint32 i = 0; i < 8; i++) {
        if (triangle_intersect_bounds(v0, v1, v2, children_[i]->GetBounds())) {
          children_[i]->AddFace(face_index);
        }
      }
    }

    // We no longer need the current node to hold any face indices as
    // its children have fully taken them over.
    face_indices_.clear();
    is_leaf_node_ = false;

    // Give the newly constructed children a chance to subdivide.
    // For a multithreaded implementation we'll simply queue the
    // subdivide operation.
    for (uint8 j = 0; j < 8; j++) {
      children_[j]->Subdivide();
    }
  }
}

uint8 BvhNode::ClosestChild(const vector3& point) const {
  vector3 trace_dir = point - aabb_.query_center();
  uint32 x_test = trace_dir.x >= 0.0f;
  uint32 y_test = trace_dir.y >= 0.0f;
  uint32 z_test = trace_dir.z >= 0.0f;
  return x_test | (z_test << 1) | (y_test << 2);
}

bool BvhNode::Trace(const ray& trajectory, BvhCollision* hit_info) const {
  collision node_hit;
  if (!ray_intersect_bounds(aabb_, trajectory, &node_hit)) {
    return false;
  }

  bool trace_result = false;

  if (IsLeafNode()) {
    // Traverse faces and return closest hit (if any)
    for (uint32 i = 0; i < face_indices_.size(); i++) {
      collision temp_hit;
      vector2 temp_bary_coords;
      uint32 face_index = face_indices_.at(i);

      const BvhFace& face = tree_faces_->at(face_index);
      const vector3& v0 = tree_vertices_->at(face.vertex_indices[0]);
      const vector3& v1 = tree_vertices_->at(face.vertex_indices[1]);
      const vector3& v2 = tree_vertices_->at(face.vertex_indices[2]);

      if (ray_intersect_triangle(v0, v1, v2, trajectory, &temp_hit,
                                 &temp_bary_coords)) {
        if (temp_hit.param < hit_info->param) {
          hit_info->param = temp_hit.param;
          hit_info->point = temp_hit.point;
          hit_info->normal = temp_hit.normal;
          hit_info->face_index = face_index;
          hit_info->bary_coords = temp_bary_coords;
          trace_result = true;
        }
      }
    }
  } else {
    return TraceInternal(node_hit, trajectory, hit_info);
  }
  return trace_result;
}

bool BvhNode::TraceInternal(const collision node_hit, const ray& trajectory,
                            BvhCollision* hit_info) const {
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

void Bvh::BuildBvh(::std::vector<vector3>* vertices,
                   ::std::vector<BvhFace>* faces) {
  if (!vertices->size() || !faces->size()) {
    return;
  }

  tree_vertices_ = vertices;
  tree_faces_ = faces;
  root_node_.reset(new BvhNode(vertices, faces));

  bounds root_bounds;
  for (uint32 i = 0; i < vertices->size(); i++) {
    root_bounds += vertices->at(i);
  }
  root_node_->SetBounds(root_bounds);

  for (uint32 i = 0; i < faces->size(); i++) {
    root_node_->AddFace(i);
  }

  printf("Starting BVH subdivision.\n");

  root_node_->Subdivide();

  printf("Completed BVH subdivision.\n");
}

bool Bvh::Trace(const ray& trajectory, BvhCollision* hit_info) const {
  if (root_node_.get()) {
    return root_node_->Trace(trajectory, hit_info);
  }
  return false;
}

const vector3 Bvh::GetCenter() {
  if (root_node_.get()) {
    return root_node_->aabb_.query_center();
  }
  return vector3();
}

}  // namespace base
