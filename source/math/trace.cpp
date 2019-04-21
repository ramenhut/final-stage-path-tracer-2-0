
#include "trace.h"

namespace base {

ray::ray(const vector3& init_start, const vector3& init_stop) {
  start = init_start;
  stop = init_stop;
  dir = stop - start;
}

float ray::length() const { return dir.length(); }

vector3 ray::direction() const { return dir; }

bool ray::has_zero_length() const { return compare_epsilon(length(), 0.0f); }

collision::collision() {
  // param can be any value > 1 to signify an invalid collision.
  param = 2.0;
}

bool collision::operator<(const collision& rhs) const {
  return (param < rhs.param);
}

bool collision::operator>(const collision& rhs) const {
  return (param > rhs.param);
}

bool collision::operator<=(const collision& rhs) const {
  return (param <= rhs.param);
}

bool collision::operator>=(const collision& rhs) const {
  return param >= rhs.param;
}
}  // namespace base