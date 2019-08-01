#include "mesh.h"

#include <math.h>
#include "math/intersect.h"
#include "math/random.h"
#include "object.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "third_party/tiny_obj_loader.h"

using namespace tinyobj;

namespace base {

static const uint32 kMaxFaceCountPerNode = 16;
static const uint32 kMaxSubdivisionDepth = 4;

MeshCollision::MeshCollision() : param(2.0), face_index(-1) {}

MeshBvhNode::MeshBvhNode(const MeshBvhDataSource& data_source) {
  tree_vertices_ = data_source.vertices;
  tree_faces_ = data_source.faces;
}

MeshBvhNode::MeshBvhNode(MeshBvhNode* parent_node) {
  depth_ = parent_node->depth_ + 1;
  parent_ = parent_node;
  is_leaf_node_ = true;
  tree_vertices_ = parent_node->tree_vertices_;
  tree_faces_ = parent_node->tree_faces_;
}

void MeshBvhNode::AllocateChild(int32 index) {
  children_[index].reset(new MeshBvhNode(this));
}

void MeshBvhNode::AddFace(uint32 index) { face_indices_.push_back(index); }

void MeshBvhNode::Subdivide() {
  if (depth_ >= kMaxSubdivisionDepth) {
    return;
  }

  if (face_indices_.size() > kMaxFaceCountPerNode) {
    // Configure bounds for the child nodes that correspond to
    // quadrants of the current node's bounds.
    ConfigureChildren();

    // Compare all of the parent node's faces against the newly
    // created child. Add faces to the child that intersect it.
    for (uint32 j = 0; j < face_indices_.size(); j++) {
      uint32 face_index = face_indices_.at(j);
      const MeshFace& face = tree_faces_->at(face_index);
      const vector3& v0 = tree_vertices_->at(face.vertex_indices[0]);
      const vector3& v1 = tree_vertices_->at(face.vertex_indices[1]);
      const vector3& v2 = tree_vertices_->at(face.vertex_indices[2]);
      for (uint32 i = 0; i < 8; i++) {
        if (triangle_intersect_bounds(v0, v1, v2, children_[i]->GetBounds())) {
          MeshBvhNode* node = static_cast<MeshBvhNode*>(children_[i].get());
          node->AddFace(face_index);
        }
      }
    }

    // We no longer need the current node to hold any face indices as
    // its children have fully taken them over.
    face_indices_.clear();

    // Give the newly constructed children a chance to subdivide.
    // For a multithreaded implementation we'll simply queue the
    // subdivide operation.
    for (uint8 j = 0; j < 8; j++) {
      children_[j]->Subdivide();
    }
  }
}

bool MeshBvhNode::Trace(const ray& trajectory, MeshCollision* hit_info) const {
  collision node_hit;
  if (!ray_intersect_bounds(aabb_, trajectory, &node_hit) ||
      node_hit.param > hit_info->param) {
    return false;
  }

  bool trace_result = false;

  if (IsLeafNode()) {
    // Traverse faces and return closest hit (if any)
    for (uint32 i = 0; i < face_indices_.size(); i++) {
      collision temp_hit;
      vector2 temp_bary_coords;
      uint32 face_index = face_indices_.at(i);

      const MeshFace& face = tree_faces_->at(face_index);
      const vector3& v0 = tree_vertices_->at(face.vertex_indices[0]);
      const vector3& v1 = tree_vertices_->at(face.vertex_indices[1]);
      const vector3& v2 = tree_vertices_->at(face.vertex_indices[2]);

      if (ray_intersect_triangle(v0, v1, v2, face.face_plane, trajectory, &temp_hit,
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

void MeshBvh::BuildBvh(::std::vector<vector3>* vertices,
                       ::std::vector<MeshFace>* faces) {
  if (!vertices->size() || !faces->size()) {
    return;
  }

  MeshBvhDataSource data = {vertices, faces};

  tree_vertices_ = vertices;
  tree_faces_ = faces;
  root_node_.reset(new MeshBvhNode(data));

  bounds root_bounds;
  for (uint32 i = 0; i < vertices->size(); i++) {
    root_bounds += vertices->at(i);
  }
  root_node_->SetBounds(root_bounds);

  for (uint32 i = 0; i < faces->size(); i++) {
    MeshFace* mesh_face = &faces->at(i);
    plane* face_plane = &mesh_face->face_plane;
    if (face_plane->x == 0 && face_plane->y == 0 && face_plane->z == 0 &&
        face_plane->w == 0) {
      vector3 p0 = vertices->at(mesh_face->vertex_indices[0]);
      vector3 p1 = vertices->at(mesh_face->vertex_indices[1]);
      vector3 p2 = vertices->at(mesh_face->vertex_indices[2]);
      // Compute face plane from normals, for any faces that lack plane info.
      vector3 normal = calculate_normal(p0, p1, p2);
      *face_plane = calculate_plane(normal, p0);
    }

    root_node_->AddFace(i);
  }

  root_node_->Subdivide();
}

bool MeshBvh::Trace(const ray& trajectory, MeshCollision* hit_info) const {
  if (root_node_.get()) {
    return root_node_->Trace(trajectory, hit_info);
  }
  return false;
}

const vector3 MeshBvh::GetCenter() const {
  if (root_node_.get()) {
    return root_node_->aabb_.query_center();
  }
  return vector3();
}

MeshObject::MeshObject(const ::std::string& filename, bool invert_normals,
                       const vector3& translation, const vector3& scale,
    const vector4& rotation) {
  // Load the object and initialize the shapes (including materials)
  ::std::string errors;
  attrib_t attributes;
  ::std::vector<shape_t> shapes;
  ::std::vector<material_t> materials;

  if (!LoadObj(&attributes, &shapes, &materials, &errors, filename.c_str(),
               nullptr, true)) {
    printf("Error loading obj file %s: %s.\n", filename.c_str(),
           errors.c_str());
    return;
  }

  if (!errors.empty()) {
    printf("Errors loading model: %s\n", errors.c_str());
  }

  bool transform_needed = false;
  matrix4 translation_mtx, scale_mtx, rotation_mtx;
  translation_mtx.identity();
  scale_mtx.identity();
  rotation_mtx.identity();

  if (translation.x || translation.y || translation.z) {
    transform_needed = true;
    translation_mtx = translation_mtx.translation(translation.x, translation.y,
                                                  translation.z);
  }

  if (scale.x || scale.y || scale.z) {
    transform_needed = true;
    scale_mtx = scale_mtx.scale(scale.x, scale.y, scale.z);
  }

  if (rotation.x || rotation.y || rotation.z) {
    transform_needed = true;
    rotation_mtx = rotation_mtx.rotation(
        rotation.w, vector3(rotation.x, rotation.y, rotation.z));
  }

  if (transform_needed) {
    matrix4 transform_mtx = translation_mtx * rotation_mtx * scale_mtx;
    for (uint32 i = 0; i < attributes.vertices.size(); i += 3) {
      vector3 temp_vertex = vector3(attributes.vertices[i + 0],
                                    attributes.vertices[i + 1],
                                    attributes.vertices[i + 2]);
      temp_vertex = vector3(transform_mtx * temp_vertex);
      aabb_ += temp_vertex;
      vertices_.emplace_back(temp_vertex);
    }
  }

  for (uint32 i = 0; i < attributes.normals.size(); i += 3) {
    vector3 temp_normal =
        vector3(attributes.normals[i + 0], attributes.normals[i + 1],
                attributes.normals[i + 2])
            .normalize();

    if (invert_normals) {
      temp_normal *= -1.0;
    }

    normals_.emplace_back(temp_normal);
  }

  for (uint32 i = 0; i < attributes.texcoords.size(); i += 2) {
    texcoords_.emplace_back(attributes.texcoords[i + 0],
                            attributes.texcoords[i + 1]);
  }

  // Condense all of the meshes from the obj file into a single shape.
  for (uint32 i = 0; i < shapes.size(); i++) {
    mesh_t* pMesh = &(shapes[i].mesh);

    uint32 j = 0;
    uint32 face_index = 0;
    while (j < pMesh->indices.size()) {
      MeshFace shape_face;

      shape_face.vertex_indices[0] = pMesh->indices[j + 2].vertex_index;
      shape_face.vertex_indices[1] = pMesh->indices[j + 1].vertex_index;
      shape_face.vertex_indices[2] = pMesh->indices[j + 0].vertex_index;

      shape_face.normal_indices[0] = pMesh->indices[j + 2].normal_index;
      shape_face.normal_indices[1] = pMesh->indices[j + 1].normal_index;
      shape_face.normal_indices[2] = pMesh->indices[j + 0].normal_index;

      shape_face.texcoord_indices[0] = pMesh->indices[j + 2].texcoord_index;
      shape_face.texcoord_indices[1] = pMesh->indices[j + 1].texcoord_index;
      shape_face.texcoord_indices[2] = pMesh->indices[j + 0].texcoord_index;

      if (pMesh->material_ids.size() > face_index) {
        shape_face.material = pMesh->material_ids.at(face_index);
      } else {
        shape_face.material = -1;
      }

      face_list.push_back(shape_face);
      j += pMesh->num_face_vertices.at(face_index);
      face_index++;
    }
  }

  shape_tree.BuildBvh(&vertices_, &face_list);
}

bool MeshObject::Trace(const ray& trajectory, ObjectCollision* hit_info) {
  MeshCollision temp_collision;
  temp_collision.param = hit_info->param;
  if (shape_tree.Trace(trajectory, &temp_collision)) {
    if (temp_collision.param <= hit_info->param) {
      hit_info->param = temp_collision.param;
      hit_info->point = temp_collision.point;
      hit_info->surface_normal = temp_collision.normal;
      hit_info->surface_material = material_.get();

      const MeshFace& face = face_list.at(temp_collision.face_index);

      if (normals_.size()) {
        // The mesh has normals so we use an interpolated vertex normal
        // for the collision normal, instead of an imprecise face normal.
        const vector3& n0 = normals_.at(face.normal_indices[0]);
        const vector3& n1 = normals_.at(face.normal_indices[1]);
        const vector3& n2 = normals_.at(face.normal_indices[2]);
        triangle_interpolate_barycentric_coeff(
            n0, n1, n2, temp_collision.bary_coords.x,
            temp_collision.bary_coords.y, &hit_info->surface_normal);
      }

      if (texcoords_.size()) {
        // The mesh has texcoords so we use them.
        const vector3& t0 = texcoords_.at(face.texcoord_indices[0]);
        const vector3& t1 = texcoords_.at(face.texcoord_indices[1]);
        const vector3& t2 = texcoords_.at(face.texcoord_indices[2]);
        vector3 output_texcoords;
        triangle_interpolate_barycentric_coeff(
            t0, t1, t2, temp_collision.bary_coords.x,
            temp_collision.bary_coords.y, &output_texcoords);
        hit_info->surface_texcoords =
            vector2(output_texcoords.x, output_texcoords.y);
      }

      // If materials_ has non-zero size, and we have a valid
      // material index for the face, then set the material.
      if (materials_.size() && face.material != -1) {
        hit_info->surface_material = materials_.at(face.material).get();
      }
      return true;
    }
  }
  return false;
}

}  // namespace base