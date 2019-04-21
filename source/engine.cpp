
#include "engine.h"
#include <thread>
#include "math/intersect.h"
#include "math/random.h"
#include "time.h"

#if _DEBUG
#define ENABLE_MULTITHREADING (0)
#else
#define ENABLE_MULTITHREADING (1)
#endif

namespace base {

const uint32 kMaximumTraceDepth = 32;
const float32 kTraceStepObjectOffset = 0.03f;

uint64 GetSystemTime() {
#if defined(BASE_PLATFORM_WINDOWS)
  return uint64(double(clock()) / CLOCKS_PER_SEC * 1000);
#elif defined(BASE_PLATFORM_MACOSX)
  timeval time;
  gettimeofday(&time, NULL);
  return (time.tv_sec * 1000) + (time.tv_usec / 1000);
#endif
}

uint64 GetElapsedTimeMs(uint64 from_time) {
  return (GetSystemTime() - from_time);
}

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

Scene::Scene() { sky_material_.reset(new LightMaterial(vector3(0, 0, 0))); }

void Scene::SetSkyMaterial(::std::shared_ptr<LightMaterial> material) {
  sky_material_ = material;
}

const vector3 Scene::SampleSky(uint32 depth, const vector3& view) {
  vector2 tex_coords = sphere_map_texcoords(view);
  return sky_material_->Sample(depth, vector3(), vector3(), view, vector3(),
                               vector3(), vector3(), vector3(), tex_coords) *
         3.0;
}

MeshObject* Scene::AddMeshObject(const ::std::string& filename,
                                 bool invert_normals,
                                 const vector3& translation,
                                 const vector3& scale) {
  object_list_.emplace_back(
      new MeshObject(filename, invert_normals, translation, scale));
  return reinterpret_cast<MeshObject*>(object_list_.back().get());
}

SphericalObject* Scene::AddSphericalObject(const vector3& origin,
                                           float32 radius) {
  object_list_.emplace_back(new SphericalObject(origin, radius));
  return reinterpret_cast<SphericalObject*>(object_list_.back().get());
}

PlanarObject* Scene::AddPlanarObject(const plane& data) {
  object_list_.emplace_back(new PlanarObject(data));
  return reinterpret_cast<PlanarObject*>(object_list_.back().get());
}

DiscObject* Scene::AddDiscObject(const vector3& origin, const vector3& normal,
                                 float32 radius) {
  object_list_.emplace_back(new DiscObject(origin, normal, radius));
  return reinterpret_cast<DiscObject*>(object_list_.back().get());
}

CubicObject* Scene::AddCubicObject(const vector3& origin, float32 width,
                                   float32 height, float32 depth) {
  object_list_.emplace_back(new CubicObject(origin, width, height, depth));
  return reinterpret_cast<CubicObject*>(object_list_.back().get());
}

bool Scene::Trace(const ray& trajectory, ObjectCollision* hit_info) {
  bool collision_detected = false;
  for (auto& i : object_list_) {
    collision_detected |= i.get()->Trace(trajectory, hit_info);
  }
  plane collision_plane =
      calculate_plane(hit_info->surface_normal, hit_info->collision_point);
  // If we've struck a back facing surface then invert our normal and
  // flag this collision as internal. This provides our materials with
  // a consistent orientation while also allowing them to handle internal
  // collisions appropriately.
  if (plane_distance(collision_plane, trajectory.start) < 0.0) {
    hit_info->surface_normal *= -1.0;
    hit_info->is_internal = true;
  }
  return collision_detected;
}

ImagePlaneCache::ImagePlaneCache(uint32 width, uint32 height) {
  invalidation_cache_.resize(width * height);
  collision_cache_.resize(width * height);
  Invalidate();
  width_ = width;
  height_ = height;
}

void ImagePlaneCache::Invalidate() {
  for (uint32 i = 0; i < width_ * height_; i++) {
    invalidation_cache_.at(i) = false;
  }
}

void ImagePlaneCache::CacheCollision(const ObjectCollision& hit, uint32 x,
                                     uint32 y) {
  collision_cache_.at(y * width_ + x) = hit;
}

ObjectCollision* ImagePlaneCache::FetchCollision(uint32 x, uint32 y) {
  if (invalidation_cache_.at(y * width_ + x)) {
    return &collision_cache_.at(y * width_ + x);
  }
  return nullptr;
}

vector3 TraceStep(const Camera& viewer, const ray* trajectory, Scene* scene,
                  vector3* hit_position, uint32 depth, uint32 x, uint32 y,
                  ImagePlaneCache* cache, TraceResult* result) {
  if (depth >= kMaximumTraceDepth) {
    return vector3(0, 0, 0);
  }

  // When fast render is enabled we only allow one bounce before
  // returning a white sky value.
  if (viewer.fast_render_enabled && depth > 1) {
    return vector3(1, 1, 1);
  }

  result->ray_count = depth + 1;

  bool needs_trace = true;
  ObjectCollision collision_info;

  if (depth == 0 && cache) {
    // The first bounce of a pixel is always the same, so we cache the
    // result and return it for all future first-bounce queries.
    ObjectCollision* cached_collision = cache->FetchCollision(x, y);
    if (cached_collision) {
      collision_info = *cached_collision;
      needs_trace = false;
    }
  }

  if (needs_trace) {
    if (!scene->Trace(*trajectory, &collision_info)) {
      if (hit_position) {
        (*hit_position) = trajectory->stop;
      }
      vector3 output = scene->SampleSky(
          depth, (trajectory->stop - trajectory->start).normalize());
      if (depth == 0) {
        result->color = output;
        result->normal = trajectory->dir.normalize();
        result->material_id = scene->GetSkyMaterial()->GetID();
        result->depth = viewer.z_far;
      }

      return output;
    }
    if (depth == 0 && cache) {
      cache->CacheCollision(collision_info, x, y);
    }
  }

  // Copy out our collision point, in case the caller needs it to compute
  // a final reflected color value.
  if (hit_position) {
    (*hit_position) = collision_info.collision_point;
  }

  vector3 view_vector =
      (collision_info.collision_point - trajectory->start).normalize();

  // From the material we gather the reflection vector to sample indirect light.
  vector3 reflection_vector = collision_info.surface_material->Reflection(
      view_vector, collision_info.surface_normal, collision_info.is_internal);

  ray reflection_ray(
      collision_info.collision_point,
      collision_info.collision_point + reflection_vector * viewer.z_far);

  // Adjust our starting position for the reflection by an epsilon, to ensure we
  // do not collide against the starting object.
  vector3 offset = reflection_vector * kTraceStepObjectOffset;
  reflection_ray.start += offset;
  reflection_ray.dir -= offset;

  vector3 indirect_origin;
  vector3 indirect_contribution;
  // Only trace further into the scene if the current material and sampling
  // vectors will actually make use of indirect light.
  if (collision_info.surface_material->WillUseIndirectLight(
          reflection_vector, collision_info.surface_normal)) {
    indirect_contribution =
        TraceStep(viewer, &reflection_ray, scene, &indirect_origin, depth + 1,
                  x, y, cache, result);
  }

  // Compute the final material contribution.
  vector3 output = collision_info.surface_material->Sample(
      depth, collision_info.collision_point, trajectory->start, view_vector,
      indirect_origin, reflection_vector, indirect_contribution,
      collision_info.surface_normal, collision_info.surface_texcoords,
      collision_info.is_internal);

  if (depth == 0) {
    result->color = output;
    result->normal = collision_info.surface_normal;
    result->material_id = collision_info.surface_material->GetID();
    result->depth = collision_info.collision_point.distance(trajectory->start);
  }

  return output;
}

void TracePixel(const Camera& viewer, Scene* scene, ray* trajectory, uint32 x,
                uint32 y, ImagePlaneCache* cache, TraceResult* result) {
  TraceStep(viewer, trajectory, scene, nullptr, 0, x, y, cache, result);
}

void TraceThreadFunction(const Camera& viewer, Scene* scene,
                         DisplayFrame* output, ImagePlaneCache* cache,
                         uint32 thread_index,
                         ::std::vector<uint32>* thread_ray_count) {
  float32 width = output->GetWidth();
  float32 height = output->GetHeight();
  float32 aspect_ratio = width / height;
#if ENABLE_MULTITHREADING
  float32 bin_height = height / ::std::thread::hardware_concurrency();
#else
  float32 bin_height = height;
#endif

  set_seed(GetSystemTime());

  float32 y_start = bin_height * thread_index;
  float32 y_stop = bin_height * (thread_index + 1);

  float32 fovy = viewer.fov_y * BASE_PI / 180.0;
  float32 fovx = 2.0 * atan(tan(fovy * 0.5) * aspect_ratio);

  vector3 forward_vector = (viewer.target - viewer.origin).normalize();
  vector3 right_vector = (vector3(0, 1, 0).cross(forward_vector)).normalize();
  vector3 up_vector = (forward_vector.cross(right_vector)).normalize();

  float32 half_proj_height = tanf(fovy * 0.5f) * viewer.z_far;
  float32 half_proj_width = tanf(fovx * 0.5f) * viewer.z_far;
  vector3 proj_origin = viewer.origin + forward_vector * viewer.z_far;

  plane focal_plane =
      calculate_plane(forward_vector * -1.0f,
                      viewer.origin + forward_vector * viewer.focal_depth);

#if ENABLE_MULTITHREADING
  if (thread_index == ::std::thread::hardware_concurrency() - 1) {
    y_stop = height;
  }
#endif

  for (float32 j = y_start; j < y_stop; j++)
    for (float32 i = 0; i < width; i++) {
      // Basic antialiasing: apply a small jitter (up to half pixel distance) to
      // our ray to help smooth out high frequency object and texel data from
      // our scene.
      float32 aa_jitter_x = random_float() - 0.5;
      float32 aa_jitter_y = random_float() - 0.5;

      float32 x_dist_from_origin =
          half_proj_width * (((i + aa_jitter_x) / (width - 1)) * 2.0 - 1.0);
      float32 y_dist_from_origin =
          half_proj_height * (((j + aa_jitter_y) / (height - 1)) * 2.0 - 1.0);
      vector3 stop = proj_origin + right_vector * x_dist_from_origin +
                     up_vector * y_dist_from_origin;
      ray trajectory(viewer.origin, stop);

      if (viewer.aperture_size > 0.0) {
        collision focal_hit;
        // Apply a simple depth of field effect:
        // 1. compute collision point of trajectory and our focal plane.
        // 2. jitter our ray start location by our aperture size.
        // 3. set our ray stop such that it passes through our focal target.
        if (ray_intersect_plane(focal_plane, trajectory, &focal_hit)) {
          // Compute a random offset for our start location, but guarantee
          // that the offset is confined to the unit circle oriented about
          // our view vector.
          float32 random_angle = random_float() * 2.0 * BASE_PI;
          float32 random_magnitude =
              sqrtf(random_float()) * viewer.aperture_size;
          vector3 random_offset =
              right_vector * cos(random_angle) * random_magnitude +
              up_vector * sin(random_angle) * random_magnitude;

          trajectory.start += random_offset;
          trajectory.stop =
              trajectory.start +
              (focal_hit.point - trajectory.start).normalize() * viewer.z_far;
          trajectory.dir = trajectory.stop - trajectory.start;
        }
      }

      TraceResult result;
      TracePixel(viewer, scene, &trajectory, i, j, cache, &result);
      thread_ray_count->at(thread_index) += result.ray_count;
      output->WritePixel(result, i, j);
    }
}

void TraceScene(const Camera& viewer, Scene* scene, DisplayFrame* output,
                ImagePlaneCache* cache) {
  static uint32 frame_counter = 0;
  uint64 frame_start_time = GetSystemTime();

  // fixme: avoid recreating threads each frame. This is fine for now because
  //        we're spending the overwhelming part of the frame elsewhere, but
  //        this should eventually be cleaned up.
#if ENABLE_MULTITHREADING
  ::std::vector<::std::thread> thread_list;
  ::std::vector<uint32> thread_ray_count;
  thread_ray_count.resize(::std::thread::hardware_concurrency());
  for (uint32 thread_idx = 0;
       thread_idx < ::std::thread::hardware_concurrency(); thread_idx++) {
    thread_ray_count[thread_idx] = 0;
    thread_list.emplace_back(&TraceThreadFunction, viewer, scene, output, cache,
                             thread_idx, &thread_ray_count);
  }

  for (auto& thread_ : thread_list) {
    thread_.join();
  }
#else
  ::std::vector<uint32> thread_ray_count;
  thread_ray_count.resize(1);
  thread_ray_count[0] = 0;
  TraceThreadFunction(viewer, scene, output, cache, 0, &thread_ray_count);
#endif

  uint32 frame_elapsed_time = GetElapsedTimeMs(frame_start_time);
  if (!viewer.fast_render_enabled) {
    float32 frame_sec = frame_elapsed_time / 1000.0f;
    uint32 total_frame_rays = 0;
    for (auto thread_ray_count : thread_ray_count) {
      total_frame_rays += thread_ray_count;
    }

    printf("Frame %i render time: %.2f sec. Mrays/sec: %.2f\n", frame_counter++,
           frame_sec, (total_frame_rays) / (1000000.0f * frame_sec));
  }

  output->SetFrameCount(frame_counter);
}

float32 TraceRange(const Camera& viewer, Scene* scene, DisplayFrame* frame,
                   float32 x, float32 y) {
  float32 width = frame->GetWidth();
  float32 height = frame->GetHeight();
  float32 aspect_ratio = width / height;

  float32 fovy = viewer.fov_y * BASE_PI / 180.0;
  float32 fovx = 2.0 * atan(tan(fovy * 0.5) * aspect_ratio);

  vector3 forward_vector = (viewer.target - viewer.origin).normalize();
  vector3 right_vector = (vector3(0, 1, 0).cross(forward_vector)).normalize();
  vector3 up_vector = (forward_vector.cross(right_vector)).normalize();

  float32 half_proj_height = tanf(fovy * 0.5f) * viewer.z_far;
  float32 half_proj_width = tanf(fovx * 0.5f) * viewer.z_far;
  vector3 proj_origin = viewer.origin + forward_vector * viewer.z_far;

  float32 x_dist_from_origin =
      half_proj_width * ((x / (width - 1)) * 2.0 - 1.0);
  float32 y_dist_from_origin =
      half_proj_height * ((y / (height - 1)) * 2.0 - 1.0);
  vector3 stop = proj_origin + right_vector * x_dist_from_origin +
                 up_vector * y_dist_from_origin;
  ray trajectory(viewer.origin, stop);

  ObjectCollision collision_info;
  if (!scene->Trace(trajectory, &collision_info)) {
    return viewer.z_far;
  }

  return (collision_info.collision_point - viewer.origin).length();
}

}  // namespace base