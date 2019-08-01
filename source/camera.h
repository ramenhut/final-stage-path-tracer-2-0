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

#ifndef __CAMERA_H__
#define __CAMERA_H__

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

}  // namespace base

#endif  // __CAMERA_H__