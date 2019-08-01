
#include "volume.h"

namespace base {

bounds::bounds() { vector_count = 0; }

void bounds::set_base(const vector3& pNewBase) {
  // Our base is simply the center point on the bottom quad
  // of our bounding box.

  float32 half_height = (bounds_max.y - bounds_min.y) * 0.5f;
  vector3 v_center = (bounds_max + bounds_min) * 0.5f;
  vector3 v_vector_to_new = (pNewBase)-v_center;

  bounds_min += v_vector_to_new;
  bounds_max += v_vector_to_new;

  bounds_min.y += half_height;
  bounds_max.y += half_height;
}

void bounds::set_center(const vector3& pNewCenter) {
  // Determine the mid-point, reset it to pTrans, and update the bounds

  vector3 v_center = (bounds_max + bounds_min) * 0.5f;
  vector3 v_vector_to_new = (pNewCenter)-v_center;

  bounds_min += v_vector_to_new;
  bounds_max += v_vector_to_new;
}

void bounds::scale(const vector3& scale) {
  vector3 half_dims = (bounds_max - bounds_min) * 0.5f;
  vector3 center = (bounds_max + bounds_min) * 0.5f;

  half_dims = half_dims * (scale);

  bounds_min = center - (half_dims);
  bounds_max = center + (half_dims);
}

void bounds::translate(const vector3& pTrans) {
  bounds_min += (pTrans);
  bounds_max += (pTrans);
}

void bounds::rotate(const vector3& vAxis, float32 fangle) {
  // Rotate but preserve axis alignment -- we accomplish this by
  // using a temporary OOBB (which is cheaper than recomputing the
  // AABB from a vertex source).

  vector3 v_center = (bounds_max + bounds_min) * 0.5f;
  vector3 v_norm_max = bounds_max - v_center;
  vector3 v_norm_min = bounds_min - v_center;

  vector3 v_box[8] = {v_norm_min,
                      vector3(v_norm_min.x, v_norm_min.y, v_norm_max.z),
                      vector3(v_norm_min.x, v_norm_max.y, v_norm_min.z),
                      vector3(v_norm_min.x, v_norm_max.y, v_norm_max.z),
                      vector3(v_norm_max.x, v_norm_min.y, v_norm_min.z),
                      vector3(v_norm_max.x, v_norm_min.y, v_norm_max.z),
                      vector3(v_norm_max.x, v_norm_max.y, v_norm_min.z),
                      v_norm_max};

  // Traverse down each, rotating it and then select extents

  clear();

  for (uint32 i = 0; i < 8; i++) {
    v_box[i] = v_box[i].rotate(fangle, vAxis);

    (*this) += v_box[i];
  }

  bounds_min = bounds_min + v_center;
  bounds_max = bounds_max + v_center;
}

bounds bounds::transform(const matrix4& pXForm) const {
  bounds temp;

  // We must expand our bounding box two eight discrete points,
  // and then transform them by our matrix.

  vector3 v_max = bounds_max;
  vector3 v_min = bounds_min;

  vector4 v_box[8] = {
      vector4(v_min.x, v_min.y, v_min.z, 1.0f),
      vector4(v_min.x, v_min.y, v_max.z, 1.0f),
      vector4(v_min.x, v_max.y, v_min.z, 1.0f),
      vector4(v_min.x, v_max.y, v_max.z, 1.0f),
      vector4(v_max.x, v_min.y, v_min.z, 1.0f),
      vector4(v_max.x, v_min.y, v_max.z, 1.0f),
      vector4(v_max.x, v_max.y, v_min.z, 1.0f),
      vector4(v_max.x, v_max.y, v_max.z, 1.0f),
  };

  // Transform each point and store bounds

  for (uint32 i = 0; i < 8; i++) {
    v_box[i] = (pXForm)*v_box[i];

    if (0.0 != v_box[i].w) {
      v_box[i] = v_box[i] / v_box[i].w;
    }

    temp += (vector3)v_box[i];
  }

  return temp;
}

void bounds::set_min(const vector3& p_min) { bounds_min = p_min; }

void bounds::set_max(const vector3& p_max) { bounds_max = p_max; }

vector3 bounds::query_center() const {
  return (bounds_min + bounds_max) * 0.5f;
}

float32 bounds::query_volume() const {
  return (bounds_max.x - bounds_min.x) * (bounds_max.y - bounds_min.y) *
         (bounds_max.z - bounds_min.z);
}

uint32 bounds::query_vector_count() const { return vector_count; }

void bounds::clear() {
  vector_count = 0;

  bounds_min = vector3(0, 0, 0);
  bounds_max = vector3(0, 0, 0);
}

bool bounds::operator==(const bounds& rhs) const {
  return bounds_min == rhs.bounds_min && bounds_max == rhs.bounds_max;
}

bool bounds::operator!=(const bounds& rhs) const { return !((*this) == rhs); }

const bounds& bounds::operator=(const bounds& rhs) {
  bounds_min = rhs.bounds_min;
  bounds_max = rhs.bounds_max;

  vector_count = rhs.vector_count;

  return (*this);
}

const bounds& bounds::operator=(const cube& rhs) {
  clear();

  for (uint32 i = 0; i < 8; i++) {
    (*this) += rhs.vertices[i];
  }

  return (*this);
}

bounds bounds::operator+(const vector3& rhs) const {
  bounds temp_bounds = (*this);

  // Update our min and max boundaries based on the value of our parameter
  if (0 == temp_bounds.vector_count) {
    temp_bounds.bounds_min = rhs;
    temp_bounds.bounds_max = rhs;
    temp_bounds.vector_count = 1;

    return temp_bounds;
  }

  temp_bounds.vector_count++;

  if (rhs[0] > temp_bounds.bounds_max[0]) temp_bounds.bounds_max[0] = rhs[0];
  if (rhs[1] > temp_bounds.bounds_max[1]) temp_bounds.bounds_max[1] = rhs[1];
  if (rhs[2] > temp_bounds.bounds_max[2]) temp_bounds.bounds_max[2] = rhs[2];

  if (rhs[0] < temp_bounds.bounds_min[0]) temp_bounds.bounds_min[0] = rhs[0];
  if (rhs[1] < temp_bounds.bounds_min[1]) temp_bounds.bounds_min[1] = rhs[1];
  if (rhs[2] < temp_bounds.bounds_min[2]) temp_bounds.bounds_min[2] = rhs[2];

  return temp_bounds;
}

const bounds& bounds::operator+=(const vector3& rhs) {
  (*this) = (*this) + rhs;

  return (*this);
}

bounds bounds::operator+(const bounds& rhs) const {
  bounds temp_bounds(*this);

  // Update our min and max boundaries based on the value of our parameter

  if (0 == temp_bounds.vector_count) {
    temp_bounds.bounds_min = rhs.bounds_min;
    temp_bounds.bounds_max = rhs.bounds_max;
    temp_bounds.vector_count = 1;

    return temp_bounds;
  }

  temp_bounds.vector_count++;

  if (rhs.bounds_max[0] > temp_bounds.bounds_max[0])
    temp_bounds.bounds_max[0] = rhs.bounds_max[0];
  if (rhs.bounds_max[1] > temp_bounds.bounds_max[1])
    temp_bounds.bounds_max[1] = rhs.bounds_max[1];
  if (rhs.bounds_max[2] > temp_bounds.bounds_max[2])
    temp_bounds.bounds_max[2] = rhs.bounds_max[2];

  if (rhs.bounds_min[0] < temp_bounds.bounds_min[0])
    temp_bounds.bounds_min[0] = rhs.bounds_min[0];
  if (rhs.bounds_min[1] < temp_bounds.bounds_min[1])
    temp_bounds.bounds_min[1] = rhs.bounds_min[1];
  if (rhs.bounds_min[2] < temp_bounds.bounds_min[2])
    temp_bounds.bounds_min[2] = rhs.bounds_min[2];

  return temp_bounds;
}

const bounds& bounds::operator+=(const bounds& rhs) {
  (*this) = (*this) + rhs;
  return (*this);
}

cube::cube() {}

cube::cube(const bounds& rhs) { (*this) = rhs; }

void cube::compute_planes() {
  face_planes[0] =
      calculate_plane(vertices[0], vertices[1], vertices[2]);  // bottom
  face_planes[1] =
      calculate_plane(vertices[6], vertices[5], vertices[4]);  // top
  face_planes[2] =
      calculate_plane(vertices[5], vertices[6], vertices[2]);  // right
  face_planes[3] =
      calculate_plane(vertices[7], vertices[4], vertices[0]);  // left
  face_planes[4] =
      calculate_plane(vertices[4], vertices[5], vertices[1]);  // front
  face_planes[5] =
      calculate_plane(vertices[6], vertices[7], vertices[3]);  // back
}

const cube& cube::operator=(const cube& rhs) {
  for (uint32 i = 0; i < 8; i++) {
    vertices[i] = rhs.vertices[i];
  }

  for (uint32 i = 0; i < 6; i++) {
    face_planes[i] = rhs.face_planes[i];
  }

  aabb_ = rhs.aabb_;
  return (*this);
}

const cube& cube::operator=(const bounds& rhs) {
  vector3 v_norm_min = rhs.bounds_min;
  vector3 v_norm_max = rhs.bounds_max;
  aabb_ = rhs;

  vector3 v_box[8] = {v_norm_min,
                      vector3(v_norm_max.x, v_norm_min.y, v_norm_min.z),
                      vector3(v_norm_max.x, v_norm_min.y, v_norm_max.z),
                      vector3(v_norm_min.x, v_norm_min.y, v_norm_max.z),
                      vector3(v_norm_min.x, v_norm_max.y, v_norm_min.z),
                      vector3(v_norm_max.x, v_norm_max.y, v_norm_min.z),
                      vector3(v_norm_max.x, v_norm_max.y, v_norm_max.z),
                      vector3(v_norm_min.x, v_norm_max.y, v_norm_max.z)};

  for (uint32 i = 0; i < 8; i++) {
    vertices[i] = v_box[i];
  }

  compute_planes();

  return (*this);
}

bool cube::operator==(const cube& rhs) const {
  for (uint32 i = 0; i < 8; i++) {
    if (vertices[i] != rhs.vertices[i]) return false;
  }

  return true;
}

bool cube::operator!=(const cube& rhs) const { return !((*this) == rhs); }

void cube::set_center(const vector3& pNewCenter) {
  // Calculate the average of the points
  vector3 v_center = query_center();
  vector3 v_vector_to_new = (pNewCenter)-v_center;

  aabb_.clear();
  for (uint32 i = 0; i < 8; i++) {
    vertices[i] = vertices[i] + v_vector_to_new;
    aabb_ += vertices[i];
  }

  compute_planes();
}

vector3 cube::query_min() const { return aabb_.bounds_min; }

vector3 cube::query_max() const { return aabb_.bounds_max; }

vector3 cube::query_center() const { return aabb_.query_center(); }

bounds cube::query_bounds() const { return aabb_; }

float32 cube::angle_x() const {
  // Flatten the x coordinate and determine angle of rotation of the quad from
  // identity.

  cube temp(*this);

  // 1. Flatten the cube into a quad along the y-axis.

  temp.vertices[4] = temp.vertices[0];
  temp.vertices[5] = temp.vertices[1];
  temp.vertices[6] = temp.vertices[2];
  temp.vertices[7] = temp.vertices[3];

  // 2. Planar map the quad to the yz axis to remove the x component.

  for (uint32 i = 0; i < 8; i++) {
    temp.vertices[i].x = 0;
  }

  // Find a characteristic vector to use for our angle calculation

  vector3 v_start = vector3(9999, 9999, 9999);
  vector3 v_end = vector3(-9999, -9999, -9999);

  for (uint32 i = 0; i < 8; i++) {
    if (temp.vertices[i].z > v_end.z) v_end = temp.vertices[i];
    if (temp.vertices[i].z < v_start.z) v_start = temp.vertices[i];
  }

  if (v_start == v_end) {
    // Our cube did not collapse into a line.
    return 0.0;
  }

  vector3 v_direction = (v_end - v_start).normalize();

  return v_direction.angle(vector3(0, 0, 1));
}

float32 cube::angle_y() const {
  // Flatten the x coordinate and determine angle of rotation of the quad from
  // identity.

  cube temp(*this);

  // 1. Flatten the cube into a quad along the z-axis.

  temp.vertices[0] = temp.vertices[3];
  temp.vertices[1] = temp.vertices[2];
  temp.vertices[4] = temp.vertices[7];
  temp.vertices[5] = temp.vertices[6];

  // 2. Planar map the quad to the xz axis to remove the y component.

  for (uint32 i = 0; i < 8; i++) {
    temp.vertices[i].y = 0;
  }

  // Find a characteristic vector to use for our angle calculation

  vector3 v_start = vector3(9999, 9999, 9999);
  vector3 v_end = vector3(-9999, -9999, -9999);

  for (uint32 i = 0; i < 8; i++) {
    if (temp.vertices[i].x > v_end.x) v_end = temp.vertices[i];
    if (temp.vertices[i].x < v_start.x) v_start = temp.vertices[i];
  }

  if (v_start == v_end) {
    // Our cube did not collapse into a line.
    return 0.0;
  }

  vector3 v_direction = (v_end - v_start).normalize();

  return v_direction.angle(vector3(1, 0, 0));
}

float32 cube::angle_z() const {
  // Flatten the x coordinate and determine angle of rotation of the quad from
  // identity.

  cube temp(*this);

  // 1. Flatten the cube into a quad along the y-axis.

  temp.vertices[4] = temp.vertices[0];
  temp.vertices[5] = temp.vertices[1];
  temp.vertices[6] = temp.vertices[2];
  temp.vertices[7] = temp.vertices[3];

  // 2. Planar map the quad to the xy axis to remove the z component.

  for (uint32 i = 0; i < 8; i++) {
    temp.vertices[i].z = 0;
  }

  // Find a characteristic vector to use for our angle calculation

  vector3 v_start = vector3(9999, 9999, 9999);
  vector3 v_end = vector3(-9999, -9999, -9999);

  for (uint32 i = 0; i < 8; i++) {
    if (temp.vertices[i].x > v_end.x) v_end = temp.vertices[i];
    if (temp.vertices[i].x < v_start.x) v_start = temp.vertices[i];
  }

  if (v_start == v_end) {
    // Our cube did not collapse into a line.
    return 0.0;
  }

  vector3 v_direction = (v_end - v_start).normalize();

  return v_direction.angle_relative(vector3(1, 0, 0), vector3(0, 0, 1));
}

void cube::rotate(const vector3& vAxis, float32 fangle) {
  vector3 v_center = query_center();
  aabb_.clear();

  for (uint32 i = 0; i < 8; i++) {
    vector3 v_temp = vertices[i] - v_center;
    vertices[i] = v_temp.rotate(fangle, vAxis) + v_center;
    aabb_ += vertices[i];
  }

  compute_planes();
}

void cube::scale(const vector3& scale) {
  vector3 center = query_center();
  aabb_.clear();

  for (uint32 i = 0; i < 8; i++) {
    vertices[i] = ((vertices[i] - center) * scale) + center;
    aabb_ += vertices[i];
  }

  compute_planes();
}

void cube::translate(const vector3& pTrans) {
  set_center(query_center() + pTrans);
}

cube cube::transform(const matrix4& pXForm) const {
  cube temp;

  // Transform each point and store bounds
  for (uint32 i = 0; i < 8; i++) {
    vector4 v_temp = vertices[i];
    v_temp.w = 1.0f;
    v_temp = (pXForm)*v_temp;

    if (0.0 != v_temp.w) {
      v_temp = v_temp / v_temp.w;
    }

    temp.vertices[i].set(v_temp.x, v_temp.y, v_temp.z);
    temp.aabb_ += temp.vertices[i];
  }

  return temp;
}

const plane& cube::query_plane(uint8 index) const { return face_planes[index]; }

}  // namespace base