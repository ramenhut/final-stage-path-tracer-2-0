
#define TINYEXR_IMPLEMENTATION
#include "third_party/tiny_exr_loader.h"

#include "bitmap.h"
#include "math/intersect.h"
#include "math/random.h"
#include "object.h"

namespace base {

const float32 kDiffuseContribThreshold = 0.001f;
const float32 kDiffuseRoughnessThreshold = 0.95f;

normal_sphere normal_generator;

void InitializeMaterials() { normal_generator.initialize(32 * 1024); }

bool matches_extension(const ::std::string &filename,
                       const ::std::string &extension) {
  return filename.size() >= extension.size() &&
         filename.compare(filename.size() - extension.size(), extension.size(),
                          extension) == 0;
}

Material::Material() { material_id_ = random_integer(); }

LightMaterial::LightMaterial(const vector3 &emissive) {
  emissive_ = emissive;
  diffuse_ = vector3(1, 1, 1);
}

bool LightMaterial::WillUseIndirectLight(const vector3 &incident_light,
                                         const vector3 &normal) const {
  return false;
}

vector3 LightMaterial::Reflection(const vector3 &view, const vector3 &normal,
                                  bool is_internal) const {
  return vector3();
}

vector3 LightMaterial::Sample(
    float32 depth, const vector3 &sample_pos, const vector3 &view_pos,
    const vector3 &view_dir, const vector3 &light_pos, const vector3 &light_dir,
    const vector3 &light_color, const vector3 &surface_normal,
    const vector2 &surface_texcoords, bool is_internal) {
  if (diffuse_map_.buffer.size() && diffuse_map_.width && diffuse_map_.height) {
    return SampleDiffuse(surface_texcoords);
  } else {
    return emissive_;
  }
}

DiffuseMaterial::DiffuseMaterial(const vector3 &diffuse) { diffuse_ = diffuse; }

DiffuseMaterial::DiffuseMaterial(const ::std::string &filename,
                                 float32 tex_scale) {
  diffuse_map_.filename = filename;
  LoadDiffuseTexture(filename, tex_scale);
}

void DiffuseMaterial::LoadDiffuseTexture(const ::std::string &filename,
                                         float32 tex_scale) {
  diffuse_map_.filename = filename;
  texture_scale_ = tex_scale;

  if (matches_extension(filename, ".bmp")) {
    LoadBitmap(filename, &diffuse_map_.buffer, &diffuse_map_.width,
               &diffuse_map_.height);
  } else if (matches_extension(filename, ".exr")) {
    float32 *output_image = nullptr;
    const char *errors = nullptr;
    int width, height;
    if (LoadEXR(&output_image, &width, &height, filename.c_str(), &errors) <
        0) {
      printf("Failed to load %s. Errors: %s.\n", filename.c_str(), errors);
      return;
    }

    printf("Loaded %s with dims: <%i, %i>.\n", filename.c_str(), width, height);
    diffuse_map_.width = width;
    diffuse_map_.height = height;
    diffuse_map_.buffer.resize(width * height * 3);

    for (uint32 i = 0; i < width * height; i++) {
      float32 *dest_ptr = &diffuse_map_.buffer.at(i * 3);
      float32 *src_ptr = output_image + i * 4;
      dest_ptr[0] = src_ptr[0];
      dest_ptr[1] = src_ptr[1];
      dest_ptr[2] = src_ptr[2];
    }
    free(output_image);
  }
}

bool DiffuseMaterial::WillUseIndirectLight(const vector3 &incident_light,
                                           const vector3 &normal) const {
  return incident_light.dot(normal) > kDiffuseContribThreshold;
}

vector3 DiffuseMaterial::Reflection(const vector3 &view, const vector3 &normal,
                                    bool is_internal) const {
  return normal_generator.random_reflection(view, normal, BASE_PI);
}

vector3 DiffuseMaterial::SampleDiffuse(const vector2 &texcoords) {
  vector3 material_diffuse = diffuse_;

  if (diffuse_map_.buffer.size() && diffuse_map_.width && diffuse_map_.height) {
    // Material has a diffuse map -- sample it for the diffuse component.
    uint32 x_tex_coord =
        (texcoords.x * texture_scale_ * diffuse_map_.width + 0.5f) - 1;
    uint32 y_tex_coord =
        (texcoords.y * texture_scale_ * diffuse_map_.height + 0.5f) - 1;

    x_tex_coord %= diffuse_map_.width;
    y_tex_coord %= diffuse_map_.height;

    uint32 pixel_offset =
        y_tex_coord * diffuse_map_.width * 3 + x_tex_coord * 3;
    material_diffuse = vector3(diffuse_map_.buffer.at(pixel_offset + 0),
                               diffuse_map_.buffer.at(pixel_offset + 1),
                               diffuse_map_.buffer.at(pixel_offset + 2));
  }
  return material_diffuse;
}

vector3 DiffuseMaterial::Sample(
    float32 depth, const vector3 &sample_pos, const vector3 &view_pos,
    const vector3 &view_dir, const vector3 &light_pos, const vector3 &light_dir,
    const vector3 &light_color, const vector3 &surface_normal,
    const vector2 &surface_texcoords, bool is_internal) {
  vector3 material_diffuse = SampleDiffuse(surface_texcoords);
  return material_diffuse * light_color *
         fmax(0.0f, surface_normal.dot(light_dir));
}

MetalMaterial::MetalMaterial(const vector3 &diffuse, float32 roughness) {
  diffuse_ = diffuse;
  roughness_ = roughness;
}

MetalMaterial::MetalMaterial(::std::string &filename, float32 roughness) {
  roughness_ = roughness;
  LoadDiffuseTexture(filename);
}

bool MetalMaterial::WillUseIndirectLight(const vector3 &incident_light,
                                         const vector3 &normal) const {
  return roughness_ <= kDiffuseRoughnessThreshold ||
         incident_light.dot(normal) > kDiffuseContribThreshold;
}

vector3 MetalMaterial::Reflection(const vector3 &view, const vector3 &normal,
                                  bool is_internal) const {
  return normal_generator.random_reflection(view, normal, BASE_PI * roughness_);
}

vector3 MetalMaterial::Sample(
    float32 depth, const vector3 &sample_pos, const vector3 &view_pos,
    const vector3 &view_dir, const vector3 &light_pos, const vector3 &light_dir,
    const vector3 &light_color, const vector3 &surface_normal,
    const vector2 &surface_texcoords, bool is_internal) {
  vector3 material_diffuse = SampleDiffuse(surface_texcoords);
  vector3 diffuse_contrib = material_diffuse * light_color *
                            fmax(0.0f, surface_normal.dot(light_dir));
  vector3 reflect_contrib = material_diffuse * light_color;
  return diffuse_contrib * roughness_ + reflect_contrib * (1.0 - roughness_);
}

MirrorMaterial::MirrorMaterial(const vector3 &diffuse) { diffuse_ = diffuse; }

bool MirrorMaterial::WillUseIndirectLight(const vector3 &incident_light,
                                          const vector3 &normal) const {
  return true;
}

vector3 MirrorMaterial::Reflection(const vector3 &view, const vector3 &normal,
                                   bool is_internal) const {
  return view.reflect(normal);
}

vector3 MirrorMaterial::Sample(
    float32 depth, const vector3 &sample_pos, const vector3 &view_pos,
    const vector3 &view_dir, const vector3 &light_pos, const vector3 &light_dir,
    const vector3 &light_color, const vector3 &surface_normal,
    const vector2 &surface_texcoords, bool is_internal) {
  return light_color * diffuse_;
}

GlassMaterial::GlassMaterial(const vector3 diffuse) { diffuse_ = diffuse; }

bool GlassMaterial::WillUseIndirectLight(const vector3 &incident_light,
                                         const vector3 &normal) const {
  return true;
}

vector3 GlassMaterial::Reflection(const vector3 &view, const vector3 &normal,
                                  bool is_internal) const {
  float32 index = 1.0f / 1.15f;
  if (is_internal) {
    index = 1.0f / index;
  }
  return view.refract(normal, index);
}

vector3 GlassMaterial::Sample(
    float32 depth, const vector3 &sample_pos, const vector3 &view_pos,
    const vector3 &view_dir, const vector3 &light_pos, const vector3 &light_dir,
    const vector3 &light_color, const vector3 &surface_normal,
    const vector2 &surface_texcoords, bool is_internal) {
  return light_color * diffuse_;
}

LiquidMaterial::LiquidMaterial(const vector3 diffuse) { diffuse_ = diffuse; }

bool LiquidMaterial::WillUseIndirectLight(const vector3 &incident_light,
                                          const vector3 &normal) const {
  return true;
}

vector3 LiquidMaterial::Reflection(const vector3 &view, const vector3 &normal,
                                   bool is_internal) const {
  if (random_float() < 0.4f) {
    return view.reflect(normal);
  }

  float32 index = 1.0f / 1.33f;
  return view.refract(normal, index);
}

// Determines the color of reflected light according to the material
// properties and the input parameters.
vector3 LiquidMaterial::Sample(
    float32 depth, const vector3 &sample_pos, const vector3 &view_pos,
    const vector3 &view_dir, const vector3 &light_pos, const vector3 &light_dir,
    const vector3 &light_color, const vector3 &surface_normal,
    const vector2 &surface_texcoords, bool is_internal) {
  return light_color * diffuse_;  // SampleDiffuse(surface_texcoords);
}

CeramicMaterial::CeramicMaterial(const vector3 &diffuse, float32 shininess) {
  diffuse_ = diffuse;
  shininess_ = shininess;
}

bool CeramicMaterial::WillUseIndirectLight(const vector3 &incident_light,
                                           const vector3 &normal) const {
  return true;
}

vector3 CeramicMaterial::Reflection(const vector3 &view_dir,
                                    const vector3 &normal,
                                    bool is_internal) const {
  if (random_float() < 0.1f) {
    return normal_generator.random_reflection(view_dir, normal, 0.0f);
  }
  return normal_generator.random_reflection(view_dir, normal,
                                            BASE_PI * (1.0 - shininess_));
}

vector3 CeramicMaterial::Sample(
    float32 depth, const vector3 &sample_pos, const vector3 &view_pos,
    const vector3 &view_dir, const vector3 &light_pos, const vector3 &light_dir,
    const vector3 &light_color, const vector3 &surface_normal,
    const vector2 &surface_texcoords, bool is_internal) {
  vector3 half_vec = ((view_dir * -1.0) + light_dir).normalize();
  vector3 diffuse_contrib = SampleDiffuse(surface_texcoords) * light_color *
                            fmax(0.0f, surface_normal.dot(light_dir));
  float32 dot_spec = pow(half_vec.dot(surface_normal), 50);
  return light_color * dot_spec + diffuse_contrib * (1.0 - dot_spec);
}

GlowMaterial::GlowMaterial(const vector3 &diffuse, const vector3 &glow,
                           float32 shininess) {
  diffuse_ = diffuse;
  glow_ = glow;
  shininess_ = shininess;
}
vector3 GlowMaterial::Sample(float32 depth, const vector3 &sample_pos,
                             const vector3 &view_pos, const vector3 &view_dir,
                             const vector3 &light_pos, const vector3 &light_dir,
                             const vector3 &light_color,
                             const vector3 &surface_normal,
                             const vector2 &surface_texcoords,
                             bool is_internal) {
  return CeramicMaterial::Sample(depth, sample_pos, view_pos, view_dir,
                                 light_pos, light_dir, light_color,
                                 surface_normal, surface_texcoords) +
         glow_;
}

FogMaterial::FogMaterial(const vector3 diffuse, float32 density) {
  diffuse_ = diffuse;
  // Density is measured in units of transparency per nm^2.
  density_ = density * 1000.0f;
}

bool FogMaterial::WillUseIndirectLight(const vector3 &incident_light,
                                       const vector3 &normal) const {
  return true;
}

vector3 FogMaterial::Reflection(const vector3 &view, const vector3 &normal,
                                bool is_internal) const {
  return view;
}

vector3 FogMaterial::Sample(float32 depth, const vector3 &sample_pos,
                            const vector3 &view_pos, const vector3 &view_dir,
                            const vector3 &light_pos, const vector3 &light_dir,
                            const vector3 &light_color,
                            const vector3 &surface_normal,
                            const vector2 &surface_texcoords,
                            bool is_internal) {
  // For the first bounce we compute a volumetric fog contribution. For all
  // further bounces we simply propagate the indirect lighting value. Fog is
  // calculated as the probability of the ray being absorbed by a fog particle.
  // The further a ray travels through the fog the higher this probability.
  if (0 == depth) {
    float32 dist = light_pos.distance(sample_pos);
    float32 threshold =
        saturate(max(0.0f, (dist * dist) * density_ * 0.00005f));
    if (random_float() < threshold) {
      return diffuse_;
    }
  }
  return light_color;
}

}  // namespace base