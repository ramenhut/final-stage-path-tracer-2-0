#include "object.h"
#include <math.h>
#include "math/intersect.h"
#include "math/random.h"

namespace base {

ObjectCollision::ObjectCollision()
    : param(2.0), surface_material(nullptr), is_internal(false) {}

void Object::SetMaterial(::std::shared_ptr<Material> material) {
  material_ = material;
}

Object::Object() {}

SphericalObject::SphericalObject(const vector3 &origin, float32 radius)
    : origin_(origin), radius_(radius) {
  aabb_ += origin + vector3(radius, radius, radius);
  aabb_ += origin - vector3(radius, radius, radius);
}

bool SphericalObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  collision temp_collision;
  if (ray_intersect_sphere(origin_, radius_, trajectory, &temp_collision)) {
    if (temp_collision.param < hit_info->param) {
      hit_info->param = temp_collision.param;
      hit_info->point = temp_collision.point;
      hit_info->surface_normal = temp_collision.normal;
      hit_info->surface_material = material_.get();
      hit_info->surface_texcoords = sphere_map_texcoords(temp_collision.normal);
      return true;
    }
  }
  return false;
}

PlanarObject::PlanarObject(const plane &data) : plane_(data) {
  vector3 normal(data[0], data[1], data[2]);
  vector3 up(0, 1, 0);
  float32 distance = -data[3];

  vector3 point_on_plane = normal * distance;
  vector3 right = normal.cross(up);
  vector3 forward = normal.cross(right);

  aabb_ += point_on_plane + right * 1000;
  aabb_ += point_on_plane - right * 1000;
  aabb_ += point_on_plane + forward * 1000;
  aabb_ += point_on_plane - forward * 1000;

  // All objects must have a non-zero bounding volume. For
  // objects specified in 2D, we adjust their bounds slightly
  // to give them thickness.
  aabb_ += point_on_plane + normal * BASE_EPSILON;
  aabb_ += point_on_plane - normal * BASE_EPSILON;
}

bool PlanarObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  collision temp_collision;
  if (ray_intersect_plane(plane_, trajectory, &temp_collision)) {
    if (temp_collision.param < hit_info->param) {
      hit_info->param = temp_collision.param;
      hit_info->point = temp_collision.point;
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

  vector3 up(0, 1, 0);
  vector3 right = normal.cross(up);
  vector3 forward = normal.cross(right);

  aabb_ += origin + right * radius;
  aabb_ += origin - right * radius;
  aabb_ += origin + forward * radius;
  aabb_ += origin - forward * radius;

  // All objects must have a non-zero bounding volume. For
  // objects specified in 2D, we adjust their bounds slightly
  // to give them thickness.
  aabb_ += origin + normal * BASE_EPSILON;
  aabb_ += origin - normal * BASE_EPSILON;
}

bool DiscObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  collision temp_collision;
  if (ray_intersect_plane(plane_, trajectory, &temp_collision)) {
    if (temp_collision.point.distance(origin_) <= radius_) {
      if (temp_collision.param < hit_info->param) {
        hit_info->param = temp_collision.param;
        hit_info->point = temp_collision.point;
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

CuboidObject::CuboidObject(const vector3 &origin, float32 width, float32 height,
                           float32 depth) {
  bounds temp_aabb;
  temp_aabb += origin - vector3(width * 0.5, height * 0.5, depth * 0.5);
  temp_aabb += origin + vector3(width * 0.5, height * 0.5, depth * 0.5);
  cube_data_ = temp_aabb;
}
void CuboidObject::Rotate(const vector3 &axis, float32 angle) {
  cube_data_.rotate(axis, angle);
}

bool CuboidObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  bool collision_detected = false;
  // Traverse the planes and determine which we collide.
  // for any that we collide, check the half-planes to see if we actually
  // intersect.
  for (uint32 i = 0; i < 6; i++) {
    collision plane_hit;
    if (ray_intersect_plane(cube_data_.query_plane(i), trajectory,
                            &plane_hit)) {
      // Only refine the collision if it's better than our current best.
      if (plane_hit.param < hit_info->param) {
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
          hit_info->param = plane_hit.param;
          hit_info->point = plane_hit.point;
          hit_info->surface_normal = plane_hit.normal;
          hit_info->surface_material = material_.get();
          hit_info->surface_texcoords =
              planar_map_texcoords(plane_hit.point, plane_hit.normal) * 0.1f;
        }
      }
    }
  }

  return collision_detected;
}

QuadObject::QuadObject(const vector3 &origin, const vector3 &normal,
                       float32 width, float32 height) {
  vector3 normalized = normal.normalize();
  plane_ = calculate_plane(normalized, origin);
  origin_ = origin;
  half_width_ = width * 0.5;
  half_height_ = height * 0.5;

  vector3 up(0, 1, 0);
  bitangent_ = normalized.cross(up);
  tangent_ = normalized.cross(bitangent_);

  aabb_ += origin + bitangent_ * half_width_;
  aabb_ += origin - bitangent_ * half_width_;
  aabb_ += origin + tangent_ * half_height_;
  aabb_ += origin - tangent_ * half_height_;

  // All objects must have a non-zero bounding volume. For
  // objects specified in 2D, we adjust their bounds slightly
  // to give them thickness.
  aabb_ += origin + normal * BASE_EPSILON;
  aabb_ += origin - normal * BASE_EPSILON;
}

QuadObject::QuadObject(const vector3 &position, const vector3 &u,
                       const vector3 &v) {
  vector3 normal = u.cross(v).normalize();
  plane_ = calculate_plane(normal, position);
  origin_ = position + u * 0.5 + v * 0.5;
  half_width_ = u.length() * 0.5;
  half_height_ = v.length() * 0.5;
  bitangent_ = u.normalize();
  tangent_ = v.normalize();

  // All objects must have a non-zero bounding volume. For
  // objects specified in 2D, we adjust their bounds slightly
  // to give them thickness.
  aabb_ += origin_ + normal * BASE_EPSILON;
  aabb_ += origin_ - normal * BASE_EPSILON;

  aabb_ += origin_ + bitangent_ * half_width_;
  aabb_ += origin_ - bitangent_ * half_width_;
  aabb_ += origin_ + tangent_ * half_height_;
  aabb_ += origin_ - tangent_ * half_height_;
}

bool QuadObject::Trace(const ray &trajectory, ObjectCollision *hit_info) {
  collision temp_collision;
  if (ray_intersect_plane(plane_, trajectory, &temp_collision)) {
    // we've struck the plane, now check to see if we're within the quad's
    // boundary.
    vector3 plane_hit = temp_collision.point - origin_;
    float32 tangent_dist = tangent_.dot(plane_hit);
    float32 bitangent_dist = bitangent_.dot(plane_hit);
    if (fabs(bitangent_dist) > half_width_ ||
        fabs(tangent_dist) > half_height_) {
      return false;
    }

    if (temp_collision.param < hit_info->param) {
      hit_info->param = temp_collision.param;
      hit_info->point = temp_collision.point;
      hit_info->surface_normal = temp_collision.normal;
      hit_info->surface_material = material_.get();
      hit_info->surface_texcoords =
          planar_map_texcoords(temp_collision.point, temp_collision.normal);
      return true;
    }
  }
  return false;
}

}  // namespace base