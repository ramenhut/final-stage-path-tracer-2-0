
#include "camera.h"

namespace base {

Camera::Camera()
    : origin(0, 0, -200),
      target(0, 0, 0),
      fov_y(45),
      aperture_size(1.5),
      focal_depth(80),
      z_near(1.0f),
      z_far(10000.0f),
      fast_render_enabled(false) {}

Camera::Camera(const vector3& new_origin, const vector3& new_target)
    : origin(new_origin),
      target(new_target),
      fov_y(45),
      aperture_size(1.5),
      focal_depth(80),
      z_near(1.0f),
      z_far(10000.0f),
      fast_render_enabled(false) {}

}  // namespace base