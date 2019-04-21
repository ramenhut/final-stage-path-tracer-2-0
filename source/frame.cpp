
#include "frame.h"

#define GAMMA_CORRECT_FRAME (1)

namespace base {

DisplayFrame::DisplayFrame(uint32 width, uint32 height)
    : width_(width), height_(height) {
  render_target_ = new vector3[width * height];
  display_buffer_ = new uint8[3 * width * height];
  count_buffer_ = new uint32[width * height];
  normal_buffer_ = new vector3[width * height];
  depth_buffer_ = new float32[width * height];
  material_id_buffer_ = new uint64[width * height];
  filtered_render_target_ = new vector3[width * height];

  if (!render_target_ || !display_buffer_ || !count_buffer_ ||
      !normal_buffer_ || !depth_buffer_ || !material_id_buffer_ ||
      !filtered_render_target_) {
    delete[] render_target_;
    delete[] display_buffer_;
    delete[] count_buffer_;
    delete[] normal_buffer_;
    delete[] depth_buffer_;
    delete[] material_id_buffer_;
    delete[] filtered_render_target_;
  }

  Reset();
}

DisplayFrame::~DisplayFrame() {
  delete[] render_target_;
  delete[] display_buffer_;
  delete[] count_buffer_;
  delete[] normal_buffer_;
  delete[] depth_buffer_;
  delete[] material_id_buffer_;
  delete[] filtered_render_target_;
}

void DisplayFrame::Reset() {
  memset(render_target_, 0, sizeof(vector3) * width_ * height_);
  memset(count_buffer_, 0, sizeof(uint32) * width_ * height_);
  memset(display_buffer_, 0, 3 * width_ * height_);
  memset(normal_buffer_, 0, sizeof(vector3) * width_ * height_);
  memset(depth_buffer_, 0, sizeof(float32) * width_ * height_);
  memset(material_id_buffer_, 0, sizeof(uint64) * width_ * height_);
  memset(filtered_render_target_, 0, sizeof(vector3) * width_ * height_);
}

void DisplayFrame::WritePixel(const vector3& pixel, uint32 x, uint32 y) {
  // Average our new pixel value into the existing render target pixel.
  uint8* display_buffer_ptr = display_buffer_ + (3 * y * width_) + (3 * x);
  uint32* current_pixel_count = count_buffer_ + y * width_ + x;
  vector3 new_pixel =
      render_target_[y * width_ + x] * (*current_pixel_count) + pixel;

  new_pixel /= ++(*current_pixel_count);
  render_target_[y * width_ + x] = new_pixel;

  {
  // Scale and clamp our floating point values (ranging 0..1) to the
  // integer range of 0..255 for display.
#if GAMMA_CORRECT_FRAME
    display_buffer_ptr[0] = 255.0 * pow(saturate(new_pixel.x), 1.0 / 2.2) + 0.5;
    display_buffer_ptr[1] = 255.0 * pow(saturate(new_pixel.y), 1.0 / 2.2) + 0.5;
    display_buffer_ptr[2] = 255.0 * pow(saturate(new_pixel.z), 1.0 / 2.2) + 0.5;
#else
    display_buffer_ptr[0] = 255.0 * saturate(new_pixel.x) + 0.5;
    display_buffer_ptr[1] = 255.0 * saturate(new_pixel.y) + 0.5;
    display_buffer_ptr[2] = 255.0 * saturate(new_pixel.z) + 0.5;
#endif
  }
}

void DisplayFrame::WritePixel(const TraceResult& result, uint32 x, uint32 y) {
  // First, write the resultant color value to our mean buffer.
  WritePixel(result.color, x, y);
  // Write our scene descriptors to the respective buffers.
  normal_buffer_[y * width_ + x] = result.normal;
  depth_buffer_[y * width_ + x] = result.depth;
  material_id_buffer_[y * width_ + x] = result.material_id;
}

}  // namespace base