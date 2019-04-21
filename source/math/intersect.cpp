
#include "intersect.h"

namespace base {

float32 area_triangle(const vector3 &p1, const vector3 &p2, const vector3 &p3) {
  // Area of a triangle = 0.5 * base * height
  vector3 baseVec = (p3) - (p1);
  vector3 heightVec = (p2) - (p1);
  vector3 basePoint = baseVec.project(heightVec);
  heightVec = (p2)-basePoint;

  return 0.5f * baseVec.length() * heightVec.length();
}

bool point_in_bounds(const bounds &bounds, const vector3 &point) {
  if (point.x >= bounds.bounds_min.x && point.x <= bounds.bounds_max.x &&
      point.y >= bounds.bounds_min.y && point.y <= bounds.bounds_max.y &&
      point.z >= bounds.bounds_min.z && point.z <= bounds.bounds_max.z) {
    return true;
  }

  return false;
}

bool bounds_intersect_bounds(const bounds &p_bounds,
                             const bounds &within_bounds) {
  if (BASE_PARAM_CHECK) {
    if (0 == p_bounds.query_volume() || 0 == within_bounds.query_volume()) {
      return false;
    }
  }

  vector3 aCenter = p_bounds.query_center();
  vector3 bCenter = within_bounds.query_center();
  vector3 aExtents = p_bounds.bounds_max - aCenter;
  vector3 bExtents = within_bounds.bounds_max - bCenter;
  vector3 abDelta = bCenter - aCenter;

  return fabs(abDelta.x) <= (aExtents.x + bExtents.x) &&
         fabs(abDelta.y) <= (aExtents.y + bExtents.y) &&
         fabs(abDelta.z) <= (aExtents.z + bExtents.z);
}

bool point_in_plane(const plane &input, const vector3 &pt) {
  return compare_epsilon(input.dot(pt) + input.w, 0.0);
}

bool plane_intersect_plane(const plane &p1, const plane &p2, ray *out_ray) {
  vector3 p1_normal(p1.x, p1.y, p1.z);
  vector3 p2_normal(p2.x, p2.y, p2.z);

  if (p1_normal.parallel(p2_normal)) {
    // Check if p1 and p2 are actually the same plane. In this case, we return
    // true and set our out_ray equal to the origin point of the plane.
    if (p1 == p2) {
      if (out_ray) {
        out_ray->start = vector3(p1.x, p1.y, p1.z) * (-p1.w);
        out_ray->stop = out_ray->start;
      }

      return true;
    }

    // Otherwise, the two planes are distinct and parallel.
    return false;
  }

  // 1. Find the direction of the intersection line.
  vector3 direction = p1_normal.cross(p2_normal);

  // 2. Derive an arbitrary point on p1 plus a vector that points towards p2.
  vector3 point_on_p1 = p1_normal * p1.w;
  vector3 direction_to_p2 = direction.cross(p1_normal);
  point_on_p1 = point_on_p1 - direction_to_p2 * 500.0f;

  // 3. Perform a simple ray-plane intersection test to determine a point on
  //    the intersection line.
  collision info;

  ray temp_ray(point_on_p1, point_on_p1 + direction_to_p2 * 1000.0f);

  if (ray_intersect_plane(p2, temp_ray, &info)) {
    // We now have a point on the intersection line, plus its direction
    if (out_ray) {
      out_ray->start = info.point - direction * 500.0f;
      out_ray->stop = out_ray->start + direction * 1000.0f;
    }

    return true;
  }

  return false;
}

void triangle_planar_map_vectors(vector3 *v0, vector3 *v1, vector3 *vp,
                                 const plane &p) {
  if (BASE_PARAM_CHECK) {
    if (!v0 || !v1 || !vp) {
      return;
    }
  }

  // Assumes the following parameters:
  //
  //   plane: the plane equation for the triangle
  //
  //   v0: a vector that spans ( b - a ) of the triangle
  //   v1: a vector that spans ( c - a ) of the triangle
  //   vp: a vector that spans ( p - a ) -- from a point on a triangle to point
  //   a of the triangle.

  plane absplane = p;
  absplane.x = fabs(absplane.x);
  absplane.y = fabs(absplane.y);
  absplane.z = fabs(absplane.z);

  if (absplane.x >= absplane.y) {
    if (absplane.x >= absplane.z)
      v0->x = v1->x = vp->x = 0;  // x is largest - map to yz plane
    else
      v0->z = v1->z = vp->z = 0;  // z is largest - map to xy plane
  } else {
    if (absplane.y >= absplane.z)
      v0->y = v1->y = vp->y = 0;  // plane.y is largest - map to xz plane
    else
      v0->z = v1->z = vp->z = 0;  // plane.z is largest - map to xy plane
  }
}

void triangle_interpolate_barycentric_coeff(const vector3 &p0,
                                            const vector3 &p1,
                                            const vector3 &p2, float32 u,
                                            float32 v, vector3 *output) {
  if (BASE_PARAM_CHECK) {
    if (!output) {
      return;
    }
  }

  // Given a set of points and barycentric coordinates, calculate the output
  // interpolated value.
  (*output) = p0 + (p1 - p0) * u + (p2 - p0) * v;
}

void triangle_find_barycentric_coeff(const vector3 &v0, const vector3 &v1,
                                     const vector3 &vp, float32 *u,
                                     float32 *v) {
  if (BASE_PARAM_CHECK) {
    if (!u || !v) {
      return;
    }
  }

  // Assumes the following parameters:
  //
  //   v0: a vector that spans ( b - a ) of the triangle
  //   v1: a vector that spans ( c - a ) of the triangle
  //   vp: a vector that spans ( p - a ) -- from a point on a triangle to point
  //   a of the triangle.
  //
  //  u,v: the output barycentric coefficients. Note that these values must
  //  satisfy the
  //       following inequalities in order for the point that was used to create
  //       'vp' to lie within the triangle boundaries:
  //
  //       ( u >= 0 ) && ( v >= 0 ) && ( u + v <= 1 )
  float32 dot00 = v0.dot(v0);
  float32 dot01 = v0.dot(v1);
  float32 dot02 = v0.dot(vp);
  float32 dot11 = v1.dot(v1);
  float32 dot12 = v1.dot(vp);
  float32 inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);

  (*u) = (dot11 * dot02 - dot01 * dot12) * inv_denom;
  (*v) = (dot00 * dot12 - dot01 * dot02) * inv_denom;
}

bool bounds_intersect_plane(const bounds &pBounds, const plane &pPlane) {
  uint32 count = 0;
  vector3 bounds_min = pBounds.bounds_min;
  vector3 bounds_max = pBounds.bounds_max;
  // If all of bounds vertices are on the same side of the plane,
  // then there is no collision.
  for (uint8 i = 0; i < 8; i++) {
    int32 x_mask = (i & 0x1);
    int32 y_mask = ((i >> 1) & 0x1);
    int32 z_mask = ((i >> 2) & 0x1);

    vector3 bounds_vertex =
        vector3(bounds_min.x * (!x_mask) + bounds_max.x * (x_mask),
                bounds_min.y * (!y_mask) + bounds_max.y * (y_mask),
                bounds_min.z * (!z_mask) + bounds_max.z * (z_mask));

    float distance = plane_distance(pPlane, bounds_vertex);

    if (compare_epsilon(distance, 0.0f)) {
      // a point lies directly on the plane, so we
      // flag this as a valid collision.
      return true;
    }

    count += (distance > 0.0f);
  }

  return (count != 0 && count != 8);
}

bool bounds_intersect_plane(const vector3 *bounds_vertices,
                            const plane &pPlane) {
  uint32 count = 0;
  // If all of bounds vertices are on the same side of the plane,
  // then there is no collision.
  for (uint8 i = 0; i < 8; i++) {
    float distance = plane_distance(pPlane, bounds_vertices[i]);
    if (compare_epsilon(distance, 0.0f)) {
      // a point lies directly on the plane, so we
      // flag this as a valid collision.
      return true;
    }

    count += (distance > 0.0f);
  }

  return (count != 0 && count != 8);
}

bool triangle_intersect_bounds(const vector3 &p1, const vector3 &p2,
                               const vector3 &p3, const bounds &pBounds) {
  // Implementation of Akenine-Moller's triangle/bounds collision test.
  static vector3 bounds_normals[3] = {vector3(1, 0, 0), vector3(0, 1, 0),
                                      vector3(0, 0, 1)};

  vector3 bounds_min = pBounds.bounds_min;
  vector3 bounds_max = pBounds.bounds_max;
  vector3 bounds_vertices[8];

  for (uint8 i = 0; i < 8; i++) {
    int32 x_mask = (i & 0x1);
    int32 y_mask = ((i >> 1) & 0x1);
    int32 z_mask = ((i >> 2) & 0x1);

    bounds_vertices[i] =
        vector3(bounds_min.x * (!x_mask) + bounds_max.x * (x_mask),
                bounds_min.y * (!y_mask) + bounds_max.y * (y_mask),
                bounds_min.z * (!z_mask) + bounds_max.z * (z_mask));
  }

  vector3 triangle_vertices[3] = {p1, p2, p3};
  vector3 triangle_edges[] = {(p1 - p2), (p2 - p3), (p3 - p1)};

  /*
  // 1. Coarse check, triangle bounds against pBounds.
  bounds triangle_bounds;
  triangle_bounds += p1;
  triangle_bounds += p2;
  triangle_bounds += p3;

  if (!bounds_intersect_bounds(triangle_bounds, pBounds)) {
    return false;
  }
  */

  // 2. Coarse check, pBounds against triangle plane.
  plane triangle_plane = calculate_plane(p1, p2, p3);
  if (!bounds_intersect_plane(bounds_vertices, triangle_plane)) {
    return false;
  }

  // 3. Fine check, search for a separating axis.
  for (uint8 i = 0; i < 3; i++)
    for (uint8 j = 0; j < 3; j++) {
      vector3 axis = triangle_edges[i].cross(bounds_normals[j]);

      float triangle_min = 9999999.0f;
      float triangle_max = -triangle_min;

      for (uint8 k = 0; k < 3; k++) {
        float proj_length = axis.projected_length(triangle_vertices[k]);

        if (proj_length < triangle_min) {
          triangle_min = proj_length;
        }

        if (proj_length > triangle_max) {
          triangle_max = proj_length;
        }
      }

      float bounds_min = 9999999.0f;
      float bounds_max = -bounds_min;

      for (uint8 k = 0; k < 8; k++) {
        float proj_length = axis.projected_length(bounds_vertices[k]);

        if (proj_length < bounds_min) {
          bounds_min = proj_length;
        }

        if (proj_length > bounds_max) {
          bounds_max = proj_length;
        }
      }

      if (bounds_max < triangle_min || bounds_min > triangle_max) {
        return false;
      }
    }

  return true;
}

bool point_in_triangle(const vector3 &p1, const vector3 &p2, const vector3 &p3,
                       const vector3 &p_point, vector2 *bary_coords) {
  vector3 normal = calculate_normal(p1, p2, p3);
  plane plane = calculate_plane(normal, p1);
  return point_in_triangle(p1, p2, p3, plane, p_point, bary_coords);
}

bool point_in_triangle(const vector3 &p1, const vector3 &p2, const vector3 &p3,
                       const plane &p, const vector3 &p_point,
                       vector2 *bary_coords) {
  if (!point_in_plane(p, p_point)) {
    return false;
  }

  vector3 v0 = (p2) - (p1);
  vector3 v1 = (p3) - (p1);
  vector3 vp = (p_point) - (p1);
  float32 u = 0;
  float32 v = 0;

  // Planar map v0, v1, vp, and then find the barycentric coordinates
  triangle_planar_map_vectors(&v0, &v1, &vp, p);
  triangle_find_barycentric_coeff(v0, v1, vp, &u, &v);

  if ((u >= 0) && (v >= 0) && (u + v <= 1)) {
    if (bary_coords) {
      bary_coords->x = u;
      bary_coords->y = v;
    }
    return true;
  }

  return false;
}

bool ray_intersect_triangle(const vector3 &p1, const vector3 &p2,
                            const vector3 &p3, const ray &p_ray,
                            collision *hit_info, vector2 *bary_coords) {
  vector3 normal = calculate_normal((p1), (p2), (p3));
  plane plane = calculate_plane(normal, (p1));
  return ray_intersect_triangle(p1, p2, p3, plane, p_ray, hit_info,
                                bary_coords);
}

bool ray_intersect_triangle(const vector3 &p1, const vector3 &p2,
                            const vector3 &p3, const plane &p, const ray &p_ray,
                            collision *hit_info, vector2 *bary_coords) {
  // If our ray has a zero vector, then we simply perform a point-in-triangle
  // intersection.
  if (p_ray.has_zero_length()) {
    if (point_in_triangle(p1, p2, p3, p, p_ray.start, bary_coords)) {
      if (hit_info) {
        // Unfortunately in this case the normal might not 'face' the ray.
        // There isn't much we can do about this since our ray is actually
        // a point.
        hit_info->param = 0.0;
        hit_info->normal = vector3(p.x, p.y, p.z);
        hit_info->point = p_ray.start;
      }

      return true;
    }

    return false;
  }

  collision possibleHit;

  // Determine the collision point using barycentric coordiantes. We first
  // determine whether the ray intersected the plane of the triangle. If it did,
  // we calculate the collision point and test its barycentric coordinates for
  // inclusion within the triangle's boundaries.
  if (!ray_intersect_plane(p, p_ray, &possibleHit)) {
    return false;
  }

  if (possibleHit.param < 0 || possibleHit.param > 1) {
    return false;
  }

  vector3 hitPointOnPlane = possibleHit.point;

  vector3 v0 = (p2) - (p1);
  vector3 v1 = (p3) - (p1);
  vector3 vp = (hitPointOnPlane) - (p1);
  float32 u = 0;
  float32 v = 0;

  // Planar map v0, v1, vp
  triangle_planar_map_vectors(&v0, &v1, &vp, p);
  triangle_find_barycentric_coeff(v0, v1, vp, &u, &v);

  if ((u >= 0) && (v >= 0) && (u + v <= 1)) {
    // We're inside the triangle
    if (hit_info) {
      (*hit_info) = possibleHit;
    }
    if (bary_coords) {
      bary_coords->x = u;
      bary_coords->y = v;
    }

    return true;
  }

  return false;
}

bool ray_intersect_sphere(const vector3 &pSphereCenter, float32 sphereRadius,
                          const ray &p_ray, collision *hit_info) {
  // If our ray is actually just a point, perform a simplified collision check.
  if (p_ray.has_zero_length()) {
    return (p_ray.start - pSphereCenter).length() <= sphereRadius;
  }

  vector3 ray_origin_vec = p_ray.start - (pSphereCenter);
  float32 a = p_ray.dir.dot(p_ray.dir);
  float32 b = 2 * (ray_origin_vec.dot(p_ray.dir));
  float32 c = ray_origin_vec.dot(ray_origin_vec) - sphereRadius * sphereRadius;

  float32 d = b * b - 4 * a * c;
  float32 t = 0;

  if (d < 0) {
    return false;
  }

  // There are only two remaining cases here:
  //
  //   1. If d == 0, our ray grazed the sphere
  //
  //   2. If d > 0, we have multiple potential collisions and will choose the
  //   closest
  //
  // In either case, we simply compute t, which will either be the closest of
  // multiple collision points, or the only collision point.

  if (ray_origin_vec.length() <= sphereRadius) {
    // ray started within sphere, so we choose the farthest t.
    t = (-b + sqrtf(d)) / (2 * a);
  } else {
    // ray originated outside the sphere, so we choose the closest t.
    t = (-b - sqrtf(d)) / (2 * a);
  }

  /*
  float32 sqrt_d = sqrtf(d);
  float32 two_a = 2 * a;
  t = (-b - sqrt_d) / two_a;
  if (t < 0) {
      t = (b - sqrt_d) / two_a;
  }
  */

  /*
  if (compare_epsilon(d, 0.0)) {
    t = -b / (2 * a);
  } else {
    t = (-b - sqrtf(d)) / (2 * a);
  }
  */

  // For our ray to have intersected the sphere in between our
  // start and end vectors: 0 <= t <= 1
  if (t < 0 || t > 1) return false;

  // ATP: t equals the parametric solution to our collision
  if (hit_info) {
    vector3 hitPos = p_ray.start + (p_ray.dir) * t;
    hit_info->param = t;
    hit_info->point = hitPos;
    hit_info->normal = (hitPos - pSphereCenter).normalize();
  }

  return true;
}

bool ray_intersect_slab(float32 start, float32 vec, float32 slab_min,
                        float32 slab_max, float32 *p_near, float32 *p_far) {
  if (BASE_PARAM_CHECK) {
    if (!p_near || !p_far) {
      return false;
    }
  }

  // Test the * Plane Pair
  if (compare_epsilon(vec, 0.0)) {
    // Our ray is parallel to the plane, Our origin must be within the
    // slabs for there to be a potential for intersection.
    if (start < slab_min || start > slab_max) {
      return false;
    }

    // The vector is parallel to our slab and contained within it. We simply
    // return true without adjusting our near and far.
    return true;
  }

  // Compute distance of our ray to each of the slab planes in terms of
  // parametric t
  float32 T1 = (slab_min - start) / vec;
  float32 T2 = (slab_max - start) / vec;

  // Ensure that T1 is the near value, and T2 is the far
  if (T1 > T2) {
    float32 Ttemp = T1;
    T1 = T2;
    T2 = Ttemp;
  }

  // We maintain the *farthest* near and the *closest* far based on the
  // calculated T values
  if (T1 > (*p_near)) (*p_near) = T1;
  if (T2 < (*p_far)) (*p_far) = T2;

  return true;
}

bool ray_intersect_bounds_worker(const bounds &p_bounds, const vector3 &pStart,
                                 const vector3 &pStop, collision *hit_info) {
  vector3 hitNormal;

  vector3 rayStart = pStart;
  vector3 rayVector = pStop - pStart;

  float32 fBestT = -BASE_INFINITY;
  float32 fnear = -BASE_INFINITY;
  float32 ffar = BASE_INFINITY;

  // Perform per-axis slab tests - note that near and far are potentially
  // updated during each call.
  if (!ray_intersect_slab(rayStart.x, rayVector.x, p_bounds.bounds_min.x,
                          p_bounds.bounds_max.x, &fnear, &ffar)) {
    return false;
  }

  if (fnear > fBestT) {
    fBestT = fnear;
    hitNormal = (rayStart.x < p_bounds.bounds_min.x ? vector3(1, 0, 0)
                                                    : vector3(-1, 0, 0));
  }

  if (!ray_intersect_slab(rayStart.y, rayVector.y, p_bounds.bounds_min.y,
                          p_bounds.bounds_max.y, &fnear, &ffar)) {
    return false;
  }

  if (fnear > fBestT) {
    fBestT = fnear;
    hitNormal = (rayStart.y < p_bounds.bounds_min.y ? vector3(0, 1, 0)
                                                    : vector3(0, -1, 0));
  }

  if (!ray_intersect_slab(rayStart.z, rayVector.z, p_bounds.bounds_min.z,
                          p_bounds.bounds_max.z, &fnear, &ffar)) {
    return false;
  }

  if (fnear > fBestT) {
    fBestT = fnear;
    hitNormal = (rayStart.z < p_bounds.bounds_min.z ? vector3(0, 0, 1)
                                                    : vector3(0, 0, -1));
  }

  // Check our final values for validity of a collision
  if (fnear > ffar) {
    // Ray did not intersect the box
    return false;
  }

  if (ffar < 0 || fnear < 0 || fnear > 1) {
    // Line segment containing the ray intersected the box, but not within the
    // region of our ray
    return false;
  }

  // ATP: We have a collision, return the collision info
  if (hit_info) {
    hit_info->param = fnear;
    hit_info->point = rayStart + rayVector * fnear;
    hit_info->normal = hitNormal;
  }

  return true;
}

bool ray_intersect_bounds(const bounds &p_bounds, const ray &p_ray,
                          collision *hit_info) {
  // If our ray is really a point, we simply perform a point_in_bounds. Note
  // that we could more elegantly handle this case naturally via the slab
  // process, but we've decided to keep this clear for now.
  if (p_ray.has_zero_length()) {
    if (point_in_bounds(p_bounds, p_ray.start)) {
      if (hit_info) {
        hit_info->param = 0.0f;
        hit_info->point = p_ray.start;
        hit_info->normal = vector3(0, 0, 0);
      }

      return true;
    }

    return false;
  }

  // If our ray begins and ends within the bounds, then it will not intersect
  // any of our slabs. In this case, we flag the scenario as a collision but are
  // unable to report normal data.
  if (point_in_bounds(p_bounds, p_ray.start) &&
      point_in_bounds(p_bounds, p_ray.stop)) {
    if (hit_info) {
      hit_info->param = 0.0f;
      hit_info->point = p_ray.start;
      hit_info->normal = vector3(0, 0, 0);
    }

    return true;
  }

  // Modified Kay, Kajiya slab intersection technique for AABBs.
  // Note that the original algorithm assumed that rays would never originate
  // from within the bounds, so we've made some adjustments below.
  if (point_in_bounds(p_bounds, p_ray.start)) {
    if (ray_intersect_bounds_worker(p_bounds, p_ray.stop, p_ray.start,
                                    hit_info)) {
      // Our inverted segment collided with the bounds, so we reverse the
      // results.
      if (hit_info) {
        hit_info->param = 1.0f - hit_info->param;
        hit_info->normal *= -1.0f;
      }

      return true;
    }

    return false;
  }

  // Our ray did not originate within the bounds, so we perform a regular check.
  return ray_intersect_bounds_worker(p_bounds, p_ray.start, p_ray.stop,
                                     hit_info);
}

bool ray_intersect_plane(const plane &p, const ray &p_ray,
                         collision *hit_info) {
    /*
  // First we check to see whether our ray begins within our plane. If it does,
  // we simply return the starting position as the collision point.
  if (point_in_plane(p, p_ray.start)) {
    if (hit_info) {
      hit_info->param = 0.0f;
      hit_info->point = p_ray.start;
      hit_info->normal = vector3(p.x, p.y, p.z);
    }

    return true;
  }

  // ATP: our ray does not begin within the plane. If our ray is really just a
  // point, then it has no chance of intersecting our plane.
  if (p_ray.has_zero_length()) {
    return false;
  }

  // Instead, perform a general ray-plane collision check.
  float32 ns =
      (p.x * p_ray.start.x + p.y * p_ray.start.y + p.z * p_ray.start.z + p.w) *
      -1.0;
  float32 ts = (p.x * p_ray.dir.x + p.y * p_ray.dir.y + p.z * p_ray.dir.z);

  // if ts is zero, then our ray is parallel to the plane (and perpendicular to
  // the plane normal). In this case, we do not register a collision.
  // Furthermore, if the ray is pointing away (positive) from the plane, we also
  // do not register a collision, regardless of the ray's origin.
  //
  // Recall that we only perform collisions on one side of a plane, and require
  // all of our planes to be outward facing.

  if (compare_epsilon(ts, 0.0)) {
    if (point_in_plane(p, p_ray.start)) {
      if (hit_info) {
        hit_info->param = 0.0f;
        hit_info->point = p_ray.start;
        hit_info->normal = vector3(p.x, p.y, p.z);
      }

      return true;
    }

    return false;
  }

  // Note: there is no need to check for ns being relatively close to zero
  // because this would indicate that our ray began on the plane -- a case
  // which we've already tested.

  // Continue with generic plane testing.

  float32 t = ns / ts;

  if (t < 0 || t > 1) {
    return false;
  }

  // ATP: We have detected a valid collision at parametric value of t

  if (hit_info) {
    hit_info->param = t;
    hit_info->point = p_ray.start + p_ray.dir * t;
    hit_info->normal = vector3(p.x, p.y, p.z);
  }

  return true;
  */

  // A faster but less stable implementation of ray-plane intersection.
  float32 ts = (p.x * p_ray.dir.x + p.y * p_ray.dir.y + p.z * p_ray.dir.z);

  if (!compare_epsilon(ts, 0.0)) {
    float32 ns = (p.x * p_ray.start.x + p.y * p_ray.start.y +
                  p.z * p_ray.start.z + p.w) *
                 -1.0;
    float32 t = ns / ts;
    if (t >= 0 && t <= 1) {
      hit_info->param = t;
      hit_info->point = p_ray.start + p_ray.dir * t;
      hit_info->normal = vector3(p.x, p.y, p.z);
      return true;
    }
  }

  return false;
}

const vector3 project_vector(const vector3 &src, const matrix4 &transform) {
  matrix4 inverse = transform.inverse();
  vector4 point(src.x, src.y, src.z, 1.0f);
  point = inverse * point;
  point /= point.w;
  return point;
}

const vector3 project_vector_with_inverse(const vector3& src, const matrix4& inverse) {
    vector4 point(src.x, src.y, src.z, 1.0f);
    point = inverse * point;
    point /= point.w;
    return point;
}

const vector3 unproject_vector(const vector3 &src, const matrix4 &transform) {
  vector4 world_coord(src.x, src.y, src.z, 1.0f);
  vector4 point = transform * world_coord;
  point /= point.w;
  return point;
}

vector2 planar_map_texcoords(const vector3 &point, const vector3 &normal) {
    // Planar map our normal to derive texcoords from a point.
    if (normal[0] > normal[1] && normal[0] > normal[2]) {
        return vector2(point[1], point[2]);
    }
    else if (normal[1] > normal[0] && normal[1] > normal[2]) {
        return vector2(point[0], point[2]);
    }
    return vector2(point[0], point[1]);
}

vector2 sphere_map_texcoords(const vector3 &normal) {
    float32 u = atan2(normal.x, normal.z) / (2 * BASE_PI) + 0.5;
    float32 v = normal.y * 0.5 + 0.5;
    return vector2(u, 1.0 - v);
}

}  // namespace base