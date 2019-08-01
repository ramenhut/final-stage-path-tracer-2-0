
#include "normal.h"
#include "random.h"

namespace base {

normal_sphere::normal_sphere() {}

void normal_sphere::initialize(int32 count) {
  // Here we're using a fairly inefficient method for generating a (roughly)
  // uniformly distributed normal sphere. See below for a method that
  // guarantees a uniform (and more efficient) distribution.
  normal_list.resize(count);

  for (int i = 0; i < count; i++) {
    vector3 solid_axis(10, 10, 10);

    while (solid_axis.length() > 1.0) {
      solid_axis = vector3(random_float_range(-1.0f, 1.0f),
                           random_float_range(-1.0f, 1.0f),
                           random_float_range(-1.0f, 1.0f));
    }

    solid_axis = solid_axis.normalize();
    normal_list.at(i) = solid_axis;
  }
}

vector3 normal_sphere::random_normal() {
  if (normal_list.empty()) {
    initialize(32 * 1024);
  }
  return normal_list[random_integer_range(0, normal_list.size() - 1)];
}

vector3 normal_sphere::random_reflection(const vector3& incident,
                                         const vector3& normal,
                                         float32 solid_angle) {
  // Two methods:
  // 1. Pick a random diffuse vector and the perfect reflection vector.
  //    Then interpolate between the two according to the solid angle.
  // 2. Pick a random diffuse vector and the perfect reflection vector.
  //    Rotate the reflection vector about the diffuse vector,
  //    according to the solid angle.

  vector3 reflect_dir = incident.reflect(normal);
  vector3 diffuse_dir = random_normal();

  if (diffuse_dir.dot(normal) < 0.0) {
    diffuse_dir = diffuse_dir * -1.0f;
  }

  float32 interp_amount = solid_angle / BASE_PI;
  vector3 reflection =
      (diffuse_dir * (interp_amount) + reflect_dir * (1.0 - interp_amount))
          .normalize();

  if (reflection.dot(normal) < 0.0) {
    reflection = reflection * -1.0f;
  }
  return reflection;
}

vector3 normal_sphere::random_refraction(const vector3& incident,
                                         const vector3& normal,
                                         float32 solid_angle, float32 index) {
  float32 half_solid_angle = solid_angle / 2.0f;
  float32 random_solid_delta =
      (random_float_range(-1.0f, 1.0f)) * half_solid_angle;
  vector3 reflect = incident - (normal * normal.dot(incident) * 2.0f);

  // Since we've been instructed to refract, we select a refraction vector (and
  // store it in our reflect variable) that is within solid angle from the
  // proper refraction angle.

  if (compare_epsilon(index, 1.0, BASE_EPSILON)) {
    // We are travelling through air or are internal to the object,
    // thus we simply continue along the incident path
    reflect = incident;
  } else {
    reflect = incident.refract(normal, index);
  }

  reflect = reflect.normalize();

  // Select a random vector to use as the axis for rotation,
  // then rotate our normal by a random value about that axis.

  vector3 solid_axis = random_normal();

  if (solid_axis.dot(normal) < 0.0) {
    solid_axis = solid_axis * -1.0f;
  }

  // Now rotate our reflection vector by our random half angle, about our random
  // axis

  if (compare_epsilon(solid_angle, BASE_PI, BASE_EPSILON)) {
    return solid_axis;
  } else if (compare_epsilon(solid_angle, 0.0, BASE_EPSILON)) {
    return reflect;
  }

  return reflect.rotate(random_solid_delta, solid_axis);
}

}  // namespace base