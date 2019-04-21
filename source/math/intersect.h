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

#ifndef __INTERSECT_H__
#define __INTERSECT_H__

#include "base.h"
#include "matrix3.h"
#include "matrix4.h"
#include "plane.h"
#include "trace.h"
#include "vector3.h"
#include "vector4.h"
#include "volume.h"

namespace base {

float area_triangle(const vector3& p1, const vector3& p2, const vector3& p3);

bool point_in_bounds(const bounds& bounds, const vector3& point);
bool point_in_plane(const plane& plane, const vector3& point);
bool point_in_triangle(const vector3& p1, const vector3& p2, const vector3& p3,
                       const vector3& p_point, vector2* bary_coords);
bool point_in_triangle(const vector3& p1, const vector3& p2, const vector3& p3,
                       const plane& p_plane, const vector3& p_point,
                       vector2* bary_coords);

void triangle_interpolate_barycentric_coeff(const vector3& p0,
                                            const vector3& p1,
                                            const vector3& p2, float32 u,
                                            float32 v, vector3* output);
void triangle_find_barycentric_coeff(const vector3& v0, const vector3& v1,
                                     const vector3& vp, float32* u, float32* v);
void triangle_planar_map_vectors(vector3* v0, vector3* v1, vector3* vp,
                                 const plane& plane);
bool triangle_intersect_bounds(const vector3& p1, const vector3& p2,
                               const vector3& p3, const bounds& pBounds);

// Ray casting

bool ray_intersect_sphere(const vector3& pSphereCenter, float32 sphereRadius,
                          const ray& p_ray, collision* hit_info);
bool ray_intersect_bounds(const bounds& pBounds, const ray& p_ray,
                          collision* hit_info);
bool ray_intersect_plane(const plane& p_plane, const ray& p_ray,
                         collision* hit_info);
bool ray_intersect_slab(float32 start, float32 vec, float32 slab_min,
                        float32 slab_max, float32* near, float32* far);
bool ray_intersect_triangle(const vector3& p1, const vector3& p2,
                            const vector3& p3, const ray& p_ray,
                            collision* hit_info, vector2* bary_coords);
bool ray_intersect_triangle(const vector3& p1, const vector3& p2,
                            const vector3& p3, const plane& p_plane,
                            const ray& p_ray, collision* hit_info,
                            vector2* bary_coords);

// Shape casting

bool plane_intersect_plane(const plane& p1, const plane& p2, ray* out_ray);
bool bounds_intersect_plane(const bounds& pBounds, const plane& pPlane);
bool bounds_intersect_bounds(const bounds& pBounds,
                             const bounds& within_bounds);
// World space coordinate -> screen coordinate.
const vector3 unproject_vector(const vector3& src, const matrix4& transform);
// Screen coordinate -> world space coordinate.
const vector3 project_vector(const vector3& src, const matrix4& transform);
// Screen coordinate -> world space coordinate, but transform is expected to be
// the inverse view projection transform.
const vector3 project_vector_with_inverse(const vector3& src, const matrix4& inverse);

// Texture coordinate mapping.

vector2 planar_map_texcoords(const vector3 &point, const vector3 &normal);
vector2 sphere_map_texcoords(const vector3 &normal);

}  // namespace base

#endif  // __INTERSECT_H__
