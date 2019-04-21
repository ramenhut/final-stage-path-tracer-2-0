
#include "statistics.h"

namespace base {

float32 compute_mean(float32* point_list, uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || 0 == count) {
      return 0.0f;
    }
  }

  float32 mean = 0;

  for (uint32 i = 0; i < count; i++) {
    mean += point_list[i];
  }

  return mean / ((float32)count);
}

float32 compute_variance(float32* point_list, uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || 0 == count) {
      return 0.0f;
    }
  }

  float32 delta_sum = 0;
  float32 mean = compute_mean(point_list, count);

  for (uint32 i = 0; i < count; i++) {
    float32 delta = point_list[i] - mean;
    delta_sum += delta * delta;
  }

  return delta_sum / ((float32)count);
}

float32 compute_deviation(float32* point_list, uint32 count) {
  return sqrtf(compute_variance(point_list, count));
}

float32 compute_covariance(float32* point_listA, float32* point_listB,
                           uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_listA || !point_listB || 0 == count) {
      return 0.0f;
    }
  }

  float32 delta_sum = 0;
  float32 meanA = compute_mean(point_listA, count);
  float32 meanB = compute_mean(point_listB, count);

  for (uint32 i = 0; i < count; i++) {
    float32 delta_a = point_listA[i] - meanA;
    float32 delta_b = point_listB[i] - meanB;
    delta_sum += delta_a * delta_b;
  }

  return delta_sum / ((float32)count);
}

vector2 compute_mean(vector2* point_list, uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || 0 == count) {
      return vector2();
    }
  }

  vector2 mean(0, 0);

  for (uint32 i = 0; i < count; i++) {
    mean += point_list[i];
  }

  return mean / ((float32)count);
}

vector2 compute_variance(vector2* point_list, uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || 0 == count) {
      return vector2();
    }
  }

  vector2 delta_sum(0, 0);
  vector2 mean = compute_mean(point_list, count);

  for (uint32 i = 0; i < count; i++) {
    vector2 delta = point_list[i] - mean;
    delta_sum += delta * delta;
  }

  return delta_sum / ((float32)count);
}

vector2 compute_deviation(vector2* point_list, uint32 count) {
  vector2 v = compute_variance(point_list, count);

  v.x = sqrtf(v.x);
  v.y = sqrtf(v.y);

  return v;
}

vector2 compute_covariance(vector2* point_listA, vector2* point_listB,
                           uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_listA || !point_listB || 0 == count) {
      return vector2();
    }
  }

  vector2 delta_sum(0, 0);
  vector2 meanA = compute_mean(point_listA, count);
  vector2 meanB = compute_mean(point_listB, count);

  for (uint32 i = 0; i < count; i++) {
    vector2 delta_a = point_listA[i] - meanA;
    vector2 delta_b = point_listB[i] - meanB;
    delta_sum += delta_a * delta_b;
  }

  return delta_sum / ((float32)count);
}

vector3 compute_mean(vector3* point_list, uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || 0 == count) {
      return vector3();
    }
  }

  vector3 mean(0, 0, 0);

  for (uint32 i = 0; i < count; i++) {
    mean += point_list[i];
  }

  return mean / ((float32)count);
}

vector3 compute_variance(vector3* point_list, uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || 0 == count) {
      return vector3();
    }
  }

  vector3 delta_sum(0, 0, 0);
  vector3 mean = compute_mean(point_list, count);

  for (uint32 i = 0; i < count; i++) {
    vector3 delta = point_list[i] - mean;
    delta_sum += delta * delta;
  }

  return delta_sum / ((float32)count);
}

vector3 compute_deviation(vector3* point_list, uint32 count) {
  vector3 v = compute_variance(point_list, count);

  v.x = sqrtf(v.x);
  v.y = sqrtf(v.y);
  v.z = sqrtf(v.z);

  return v;
}

vector3 compute_covariance(vector3* point_listA, vector3* point_listB,
                           uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_listA || !point_listB || 0 == count) {
      return vector3();
    }
  }

  vector3 delta_sum(0, 0, 0);
  vector3 meanA = compute_mean(point_listA, count);
  vector3 meanB = compute_mean(point_listB, count);

  for (uint32 i = 0; i < count; i++) {
    vector3 delta_a = point_listA[i] - meanA;
    vector3 delta_b = point_listB[i] - meanB;
    delta_sum += delta_a * delta_b;
  }

  return delta_sum / ((float32)count);
}

vector4 compute_mean(vector4* point_list, uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || 0 == count) {
      return vector4();
    }
  }

  vector4 mean(0, 0, 0, 0);

  for (uint32 i = 0; i < count; i++) {
    mean += point_list[i];
  }

  return mean / ((float32)count);
}

vector4 compute_variance(vector4* point_list, uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_list || 0 == count) {
      return vector4();
    }
  }

  vector4 delta_sum(0, 0, 0, 0);
  vector4 mean = compute_mean(point_list, count);

  for (uint32 i = 0; i < count; i++) {
    vector4 delta = point_list[i] - mean;
    delta_sum += delta * delta;
  }

  return delta_sum / ((float32)count);
}

vector4 compute_deviation(vector4* point_list, uint32 count) {
  vector4 v = compute_variance(point_list, count);

  v.x = sqrtf(v.x);
  v.y = sqrtf(v.y);
  v.z = sqrtf(v.z);
  v.w = sqrtf(v.w);

  return v;
}

vector4 compute_covariance(vector4* point_listA, vector4* point_listB,
                           uint32 count) {
  if (BASE_PARAM_CHECK) {
    if (!point_listA || !point_listB || 0 == count) {
      return vector4();
    }
  }

  vector4 delta_sum(0, 0, 0, 0);
  vector4 meanA = compute_mean(point_listA, count);
  vector4 meanB = compute_mean(point_listB, count);

  for (uint32 i = 0; i < count; i++) {
    vector4 delta_a = point_listA[i] - meanA;
    vector4 delta_b = point_listB[i] - meanB;
    delta_sum += delta_a * delta_b;
  }

  return delta_sum / ((float32)count);
}

}  // namespace base