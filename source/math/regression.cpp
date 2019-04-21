
#include "statistics.h"

namespace base {

void compute_linear_squares(vector2* point_list, uint32 count,
                            vector2* out_start, vector2* out_end) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || !out_start || !out_end) {
      return;
    }
  }

  float32 m = 0;
  float32 b = 0;
  float32 xiyi = 0;
  float32 xi2 = 0;
  float32 yi = 0;
  float32 xi = 0;

  for (uint32 j = 0; j < count; j++) {
    xiyi += point_list[j].x * point_list[j].y;
    xi2 += point_list[j].x * point_list[j].x;
    yi += point_list[j].y;
    xi += point_list[j].x;
  }

  // Solve our linear system of equations

  matrix2 A;
  vector2 B;
  vector2 C;

  A.set(xi2, xi, xi, 1);
  B.set(xiyi, yi);

  if (!A.is_invertible()) {
    return;
  }

  C = A.inverse() * B;

  m = C.x;
  b = C.y;

  // Direct method (provided for example purposes only)
  //
  // b = ( xiyi / xi2 - yi / xi ) / ( i / xi - xi / xi2 );
  // m = ( 2 * xiyi + 2 * b * xi ) / ( 2 * xi2 );
  //
  // Now we must determine an origin and vector for the line. We
  // search for the coordinate with the minimum x, and then plug this
  // into our line equation to determine the closest origin.
  //
  // We will also search for the coordinate with the maximum x, and
  // plug this into our equation to derive the line segment end point.

  float32 min_x_value = 9999999.0f;
  float32 max_x_value = 0;

  for (uint32 j = 0; j < count; j++) {
    if (point_list[j].x <= min_x_value) min_x_value = point_list[j].x;
    if (point_list[j].x >= max_x_value) max_x_value = point_list[j].x;
  }

  out_start->x = min_x_value;
  out_start->y = m * min_x_value + b;
  out_end->x = max_x_value;
  out_end->y = m * max_x_value + b;
}

void compute_linear_squares(vector3* point_list, uint32 count,
                            vector3* out_start, vector3* out_end) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || !out_start || !out_end) {
      return;
    }
  }

  std::vector<vector2> a_list;
  std::vector<vector2> b_list;

  a_list.resize(count);
  b_list.resize(count);

  for (uint32 i = 0; i < count; i++) {
    a_list[i].x = point_list[i].x;
    a_list[i].y = point_list[i].y;
    b_list[i].x = point_list[i].y;
    b_list[i].y = point_list[i].z;
  }

  vector2 a_list_origin, a_list_vector;
  vector2 b_list_origin, b_list_vector;

  compute_linear_squares(&a_list[0], count, &a_list_origin, &a_list_vector);
  compute_linear_squares(&b_list[0], count, &b_list_origin, &b_list_vector);

  out_start->x = a_list_origin.x;
  out_start->y = a_list_origin.y;
  out_start->z = b_list_origin.y;

  out_end->x = a_list_vector.x;
  out_end->y = a_list_vector.y;
  out_end->z = b_list_vector.y;
}

void compute_linear_squares(vector4* point_list, uint32 count,
                            vector4* out_start, vector4* out_end) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || !out_start || !out_end) {
      return;
    }
  }

  std::vector<vector3> a_list;
  std::vector<vector3> b_list;

  a_list.resize(count);
  b_list.resize(count);

  for (uint32 i = 0; i < count; i++) {
    a_list[i].x = point_list[i].x;
    a_list[i].y = point_list[i].y;
    a_list[i].z = point_list[i].z;
    b_list[i].x = point_list[i].y;
    b_list[i].y = point_list[i].z;
    b_list[i].z = point_list[i].w;
  }

  vector3 a_list_origin, a_list_vector;
  vector3 b_list_origin, b_list_vector;

  compute_linear_squares(&a_list[0], count, &a_list_origin, &a_list_vector);
  compute_linear_squares(&b_list[0], count, &b_list_origin, &b_list_vector);

  out_start->x = a_list_origin.x;
  out_start->y = a_list_origin.y;
  out_start->z = a_list_origin.z;
  out_start->w = b_list_origin.z;

  out_end->x = a_list_vector.x;
  out_end->y = a_list_vector.y;
  out_end->z = a_list_vector.z;
  out_end->w = b_list_vector.z;
}

}  // namespace base