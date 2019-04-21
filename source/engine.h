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

#ifndef __ENGINE_H__
#define __ENGINE_H__

#include <string>
#include "frame.h"
#include "math/base.h"
#include "material.h"
#include "object.h"

namespace base {

typedef struct Camera {
  // Distance from origin to the near clipping plane.
  float32 z_near;
  // Distance from the origin to the far clipping plane.
  float32 z_far;
  // The position of the camera. 
  vector3 origin;
  // The focus target of the camera. 
  vector3 target;
  // Camera zoom. 
  float32 fov_y;
  // Controls the range of focus. 
  float32 aperture_size;
  // Controls the depth of focus.
  float32 focal_depth;
  // Toggles fast render mode (for real-time interaction).
  bool fast_render_enabled;

  Camera();
  Camera(const vector3& new_origin, const vector3& new_target);
} Camera;

class Scene {
 public:
  // Loads a mesh object from a file and adds it to the scene.
  // Returns a pointer to the newly added object.
  MeshObject* AddMeshObject(const ::std::string& filename,
                            bool invert_normals = false,
                            const vector3& translation = vector3(0, 0, 0),
                            const vector3& scale = vector3(1, 1, 1));
  // Adds a spherical object to the scene. Returns a pointer to the new object.
  SphericalObject* AddSphericalObject(const vector3& origin, float32 radius);
  // Adds a disc object to the scene. Returns a pointer to the new object.
  DiscObject* AddDiscObject(const vector3& origin, const vector3& normal,
                            float32 radius);
  // Adds a planar object to the scene. Returns a pointer to the new object.
  PlanarObject* AddPlanarObject(const plane& data);
  // Adds a cubic object to the scene. Returns a pointer to the new object.
  CubicObject* AddCubicObject(const vector3& origin, float32 width,
                              float32 height, float32 depth);
  // Sets the default sky material for the scene.
  void SetSkyMaterial(::std::shared_ptr<LightMaterial> material);
  // Retrieves the sky material.
  LightMaterial* GetSkyMaterial() { return sky_material_.get(); }

  // Traces a ray through the scene and determines collision info.
  // Returns true if a collision was detected. False otherwise.
  bool Trace(const ray& trajectory, ObjectCollision* hit_info);
  // Returns the sky color given a view direction.
  const vector3 SampleSky(uint32 depth, const vector3& view);

  Scene();

 private:
  // The sky material.
  ::std::shared_ptr<LightMaterial> sky_material_;
  // List of objects in the scene.
  ::std::vector<::std::unique_ptr<Object>> object_list_;
};

// Caches the first collision for each pixel on the image plane.
class ImagePlaneCache {
public:
    ImagePlaneCache(uint32 width, uint32 height);
    // Resets the cache and invalidates all entries.
    void Invalidate();
    // Caches a collision at a specific pixel.
    void CacheCollision(const ObjectCollision &hit, uint32 x, uint32 y);
    // Fetches a cached collision at a given pixel. Returns nullptr if
    // the pixel does not have a cached entry.
    ObjectCollision* FetchCollision(uint32 x, uint32 y);
private:
    // Cache of previously computed per-pixel scene collisions.
    ::std::vector<ObjectCollision> collision_cache_;
    // Stores true if the given pixel is cached, false otherwise.
    ::std::vector<bool> invalidation_cache_;
    // Dimensions of the cache.
    uint32 width_;
    uint32 height_;
};

// Returns the distance to the object hit at a particular pixel.
float32 TraceRange(const Camera& viewer, Scene* scene, DisplayFrame* frame,
                   float32 x, float32 y);

// Traces the scene from the perspective of view, and deposits the results
// in the output frame. This method will never clear the output frame, so
// it is the responsibility of the caller to coordinate changes of frame.
void TraceScene(const Camera& view, Scene* scene, DisplayFrame* output, ImagePlaneCache *cache = nullptr);

}  // namespace base

#endif  // __ENGINE_H__