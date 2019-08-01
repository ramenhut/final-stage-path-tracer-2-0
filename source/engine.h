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
#include "camera.h"
#include "frame.h"
#include "material.h"
#include "math/base.h"
#include "object.h"
#include "scene.h"

namespace base {

// Caches the first collision for each pixel on the image plane.
class ImagePlaneCache {
 public:
  ImagePlaneCache(uint32 width, uint32 height);
  // Resets the cache and invalidates all entries.
  void Invalidate();
  // Caches a collision at a specific pixel.
  void CacheCollision(const ObjectCollision& hit, uint32 x, uint32 y);
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
void TraceScene(const Camera& view, Scene* scene, DisplayFrame* output,
                ImagePlaneCache* cache = nullptr);

}  // namespace base

#endif  // __ENGINE_H__