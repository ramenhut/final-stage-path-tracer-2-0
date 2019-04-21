#include "math/intersect.h"
#include "math/random.h"
#include "object.h"

#include <math.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include "third_party/tiny_obj_loader.h"

#define SCALE_MESH_VERTICES (1)
#define TARGET_MESH_SCALE (20.0f)

using namespace tinyobj;

namespace base {

ObjectCollision::ObjectCollision()
    : collision_param(2.0), surface_material(nullptr), is_internal(false) {}

void Object::SetMaterial(::std::shared_ptr<Material> material) {
  material_ = material;
}

Object::Object() {
}

SphericalObject::SphericalObject(const vector3 &origin, float32 radius)
    : origin_(origin), radius_(radius) {}

bool SphericalObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  collision temp_collision;
  if (ray_intersect_sphere(origin_, radius_, trajectory, &temp_collision)) {
    if (temp_collision.param < hit_info->collision_param) {
      hit_info->collision_param = temp_collision.param;
      hit_info->collision_point = temp_collision.point;
      hit_info->surface_normal = temp_collision.normal;
      hit_info->surface_material = material_.get();
      hit_info->surface_texcoords =
          sphere_map_texcoords(temp_collision.normal);
      return true;
    }
  }
  return false;
}

PlanarObject::PlanarObject(const plane &data) : plane_(data) {}

bool PlanarObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  collision temp_collision;
  if (ray_intersect_plane(plane_, trajectory, &temp_collision)) {
    if (temp_collision.param < hit_info->collision_param) {
      hit_info->collision_param = temp_collision.param;
      hit_info->collision_point = temp_collision.point;
      hit_info->surface_normal = temp_collision.normal;
      hit_info->surface_material = material_.get();
      hit_info->surface_texcoords =
          planar_map_texcoords(temp_collision.point, temp_collision.normal);
      return true;
    }
  }
  return false;
}

DiscObject::DiscObject(const vector3 &origin, const vector3 &normal,
                       float32 radius) {
  origin_ = origin;
  plane_ = calculate_plane(normal, origin);
  radius_ = radius;
}

bool DiscObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  collision temp_collision;
  if (ray_intersect_plane(plane_, trajectory, &temp_collision)) {
    if (temp_collision.point.distance(origin_) <= radius_) {
      if (temp_collision.param < hit_info->collision_param) {
        hit_info->collision_param = temp_collision.param;
        hit_info->collision_point = temp_collision.point;
        hit_info->surface_normal = temp_collision.normal;
        hit_info->surface_material = material_.get();
        hit_info->surface_texcoords =
            planar_map_texcoords(temp_collision.point, temp_collision.normal);
        return true;
      }
    }
  }
  return false;
}

CubicObject::CubicObject(const vector3 &origin, float32 width, float32 height,
                         float32 depth) {
  bounds temp_aabb;
  temp_aabb += origin - vector3(width * 0.5, height * 0.5, depth * 0.5);
  temp_aabb += origin + vector3(width * 0.5, height * 0.5, depth * 0.5);
  cube_data_ = temp_aabb;
}
void CubicObject::Rotate(const vector3 &axis, float32 angle) {
  cube_data_.rotate(axis, angle);
}

bool CubicObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  bool collision_detected = false;
  // Traverse the planes and determine which we collide.
  // for any that we collide, check the half-planes to see if we actually
  // intersect.
  for (uint32 i = 0; i < 6; i++) {
    collision plane_hit;
    if (ray_intersect_plane(cube_data_.query_plane(i), trajectory,
                            &plane_hit)) {
      // Only refine the collision if it's better than our current best.
      if (plane_hit.param < hit_info->collision_param) {
        bool plane_hit_detected = true;
        // Determine if we actually hit a face of the cube, and not simply
        // the face's plane. We test every adjacent plane against the hit point.
        for (uint32 j = 0; j < 6; j++) {
          // We skip the current plane (already know it's a hit), and the
          // opposite plane (whose index will be +-1 from i).
          if (i / 2 != j / 2) {
            if (plane_distance(cube_data_.query_plane(j), plane_hit.point) >
                0.0f) {
              plane_hit_detected = false;
              break;
            }
          }
        }
        if (plane_hit_detected) {
          collision_detected = true;
          hit_info->collision_param = plane_hit.param;
          hit_info->collision_point = plane_hit.point;
          hit_info->surface_normal = plane_hit.normal;
          hit_info->surface_material = material_.get();
          hit_info->surface_texcoords =
              planar_map_texcoords(plane_hit.point, plane_hit.normal);
        }
      }
    }
  }

  return collision_detected;
}

MeshObject::MeshObject(const ::std::string &filename, bool invert_normals,
                       const vector3 &translation, const vector3 &scale) {
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

  printf(
      "Loaded %s with %i vertices, %i normals, %i texcoords, and %i "
      "materials.\n",
      filename.c_str(), attributes.vertices.size(), attributes.normals.size(),
      attributes.texcoords.size(), materials.size());

#if SCALE_MESH_VERTICES
  // Adjust vertices so that the mesh fits tge desired bounding box.
  bounds temp_bb;
  for (uint32 i = 0; i < attributes.vertices.size(); i += 3) {
    vector3 temp_vert(attributes.vertices[i + 0], attributes.vertices[i + 1],
                      attributes.vertices[i + 2]);
    temp_bb += temp_vert;
  }

  printf("Mesh has bounds dimensions: %.2f x %.2f x %.2f\n",
         temp_bb.bounds_max.x - temp_bb.bounds_min.x,
         temp_bb.bounds_max.y - temp_bb.bounds_min.y,
         temp_bb.bounds_max.z - temp_bb.bounds_min.z);

  vector3 mesh_center = temp_bb.query_center();
  printf("Mesh is centered at: %.2f, %.2f, %.2f\n", mesh_center.x,
         mesh_center.y, mesh_center.z);

  float32 scale_factor = sqrt(TARGET_MESH_SCALE * TARGET_MESH_SCALE * 3) /
                         (temp_bb.bounds_max - temp_bb.query_center()).length();

  printf("Applying a scale factor of %.2f\n", scale_factor);
  for (uint32 i = 0; i < attributes.vertices.size(); i += 3) {
    attributes.vertices[i + 0] *= scale_factor * scale.x;
    attributes.vertices[i + 1] *= scale_factor * scale.y;
    attributes.vertices[i + 2] *= scale_factor * scale.z;
  }
#endif

  for (uint32 i = 0; i < attributes.vertices.size(); i += 3) {
    vector3 temp_vertex =
        vector3(attributes.vertices[i + 0], attributes.vertices[i + 1],
                attributes.vertices[i + 2]) +
        translation;
    vertices_.emplace_back(temp_vertex);
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

  printf("Shape count: %i\n", shapes.size());

  // Condense all of the meshes from the obj file into a single shape.
  for (uint32 i = 0; i < shapes.size(); i++) {
    mesh_t *pMesh = &(shapes[i].mesh);

    printf("Shape %i has %i faces and %i material indices.\n", i,
           pMesh->num_face_vertices.size(), pMesh->material_ids.size());

    uint32 j = 0;
    uint32 face_index = 0;
    while (j < pMesh->indices.size()) {
      BvhFace shape_face;

      if (pMesh->num_face_vertices.at(face_index) != 3) {
        printf("Face is non-triangular!\n");
      }

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

bool MeshObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  BvhCollision temp_collision;
  if (shape_tree.Trace(trajectory, &temp_collision)) {
    if (temp_collision.param <= hit_info->collision_param) {
      hit_info->collision_param = temp_collision.param;
      hit_info->collision_point = temp_collision.point;
      hit_info->surface_normal = temp_collision.normal;
      hit_info->surface_material = material_.get();

      const BvhFace &face = face_list.at(temp_collision.face_index);

      if (normals_.size()) {
        // The mesh has normals so we use an interpolated vertex normal
        // for the collision normal, instead of an imprecise face normal.
        const vector3 &n0 = normals_.at(face.normal_indices[0]);
        const vector3 &n1 = normals_.at(face.normal_indices[1]);
        const vector3 &n2 = normals_.at(face.normal_indices[2]);
        triangle_interpolate_barycentric_coeff(
            n0, n1, n2, temp_collision.bary_coords.x,
            temp_collision.bary_coords.y, &hit_info->surface_normal);
      }

      if (texcoords_.size()) {
        // The mesh has texcoords so we use them.
        const vector3 &t0 = texcoords_.at(face.texcoord_indices[0]);
        const vector3 &t1 = texcoords_.at(face.texcoord_indices[1]);
        const vector3 &t2 = texcoords_.at(face.texcoord_indices[2]);
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