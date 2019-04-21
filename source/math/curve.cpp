
#include "curve.h"

namespace base {

int8 evaluate_linear_bezier(const vector3& p0, const vector3& p1, float32 t,
                            vector3* output) {
  if (BASE_PARAM_CHECK) {
    if (!output) {
      return -1;
    }
  }

  (*output) = (p0) * (1.0 - t) + (p1)*t;

  return 0;
}

int8 evaluate_quadratic_bezier(const vector3& p0, const vector3& p1,
                               const vector3& p2, float32 t, vector3* output) {
  if (BASE_PARAM_CHECK) {
    if (!output) {
      return -1;
    }
  }

  vector3 pseudo0, pseudo1;

  evaluate_linear_bezier(p0, p1, t, &pseudo0);
  evaluate_linear_bezier(p1, p2, t, &pseudo1);

  (*output) = pseudo0 * (1.0 - t) + pseudo1 * t;

  return 0;
}

int8 evaluate_cubic_bezier(const vector3& p0, const vector3& p1,
                           const vector3& p2, const vector3& p3, float32 t,
                           vector3* output) {
  if (BASE_PARAM_CHECK) {
    if (!output) {
      return -1;
    }
  }

  vector3 pseudo0, pseudo1;

  evaluate_quadratic_bezier(p0, p1, p2, t, &pseudo0);
  evaluate_quadratic_bezier(p1, p2, p3, t, &pseudo1);

  (*output) = pseudo0 * (1.0 - t) + pseudo1 * t;

  return 0;
}

curve::curve() {}

curve::curve(const curve& rhs) { assign(rhs); }

curve::~curve() {}

void curve::add_point(const vector3& input) { control_list.push_back(input); }

int8 curve::evaluate(float32 fT, vector3* output) {
  if (BASE_PARAM_CHECK) {
    if (!output) {
      return -1;
    }
  }

  if (control_list.size() < 2) {
    return -1;
  }

  if (fT < 0.0) fT = 0.0;
  if (fT > 1.0) fT = 1.0;

  std::vector<vector3>::iterator p = control_list.begin();

  switch (control_list.size()) {
    case 2:
      return evaluate_linear_bezier(*p, *(p + 1), fT, output);
    case 3:
      return evaluate_quadratic_bezier(*p, *(p + 1), *(p + 2), fT, output);
    case 4:
      return evaluate_cubic_bezier(*p, *(p + 1), *(p + 2), *(p + 3), fT,
                                   output);
  }

  return -1;
}

void curve::assign(const curve& input) { control_list = input.control_list; }

const curve& curve::operator=(const curve& rhs) {
  assign(rhs);
  return (*this);
}

bool curve::operator==(const curve& rhs) const {
  return control_list == rhs.control_list;
}

bool curve::operator!=(const curve& rhs) const { return !((*this) == rhs); }

path::path() {}

path::~path() {}

void path::add_curve(const curve& input) { curve_list.push_back(input); }

uint32 path::query_occupancy() const { return curve_list.size(); }

int8 path::evaluate(float32 fT, vector3* output) {
  if (BASE_PARAM_CHECK) {
    if (!output) {
      return -1;
    }
  }

  if (fT < 0.0) fT = 0.0;
  if (fT > 1.0) fT = 1.0;

  if (0 == curve_list.size()) {
    return 0;
  }

  uint32 curve_index = (uint32)(fT * (curve_list.size()));

  //
  // if 1.0 == fT, we can overflow our curve count, so we adjust
  //

  if (curve_index >= curve_list.size()) {
    curve_index = curve_list.size() - 1;
  }

  float32 increment_length = 1.0f / ((float32)curve_list.size());
  fT = (fT - increment_length * curve_index) / increment_length;

  return curve_list[curve_index].evaluate(fT, output);
}

void path::assign(const path& input) { curve_list = input.curve_list; }

}  // namespace base