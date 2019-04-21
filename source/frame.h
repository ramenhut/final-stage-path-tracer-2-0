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

#ifndef __FRAME_H__
#define __FRAME_H__

#include "math/base.h"
#include "math/vector3.h"

namespace base {

typedef struct TraceResult {
  vector3 color;
  vector3 normal;
  float32 depth;
  uint64 material_id;
  uint64 ray_count;
  TraceResult() : depth(0.0f), material_id(0), ray_count(0) {}
} TraceResult;

class DisplayFrame {
 public:
  DisplayFrame(uint32 width, uint32 height);
  ~DisplayFrame();
  // Clears internal buffers to black (0), and resets pixel counts.
  // Call this when beginning a new frame.
  void Reset();
  // Writes pixel into the render buffer, increments count, and writes the
  // output pixel.
  void WritePixel(const vector3& pixel, uint32 x, uint32 y);
  // Incorporates a trace result into a final pixel value.
  void WritePixel(const TraceResult& result, uint32 x, uint32 y);
  // Returns a pointer to the current output buffer.
  uint8* GetDisplayBuffer() { return display_buffer_; }
  // Returns the width of the frame.
  uint32 GetWidth() { return width_; }
  // Returns the height of the frame.
  uint32 GetHeight() { return height_; }
  // Returns width / height.
  float32 GetAspectRatio() { return (float32) width_ / height_; }
  // Sets the current frame count.
  void SetFrameCount(uint32 count) { frame_count_ = count; }

 private:
  uint32 frame_count_;
  // Width of the frame, in pixels.
  uint32 width_;
  // Height of the frame, in pixels.
  uint32 height_;
  // Intermediary floating point frame buffer. May include HDR values.
  vector3* render_target_;
  // Intermediary filtered frame buffer.
  vector3* filtered_render_target_;
  // Final frame buffer used for display.
  uint8* display_buffer_;
  // Running count of samples for each pixel.
  uint32* count_buffer_;
  // Per-pixel normals for the scene.
  vector3* normal_buffer_;
  // Per-pixel depth values for the scene.
  float32* depth_buffer_;
  // Per-pixel material ids for the scene.
  uint64* material_id_buffer_;
};

}  // namespace base

#endif  // __FRAME_H__