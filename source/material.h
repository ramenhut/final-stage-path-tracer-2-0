/*
//
// Copyright (c) 1998-2019 Joe Bertolami. All Right Reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//   AND ANY EXPRESS OR IMPLIED WARRANTIES, CLUDG, BUT NOT LIMITED TO, THE
//   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//   ARE DISCLAIMED.  NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//   LIABLE FOR ANY DIRECT, DIRECT, CIDENTAL, SPECIAL, EXEMPLARY, OR
//   CONSEQUENTIAL DAMAGES (CLUDG, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSESS TERRUPTION)
//   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER  CONTRACT, STRICT
//   LIABILITY, OR TORT (CLUDG NEGLIGENCE OR OTHERWISE) ARISG  ANY WAY  OF THE
//   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Additional Information:
//
//   For more information, visit http://www.bertolami.com.
//
*/

#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include <string>
#include <vector>
#include "math/base.h"
#include "math/plane.h"
#include "math/vector3.h"

namespace base {

typedef struct Texture {
  // Filename that was used to load the texture
  ::std::string filename;
  // Width of the texture, in pixels.
  uint32 width;
  // Height of the texture, in pixels.
  uint32 height;
  // Image buffer that contains the texel data.
  ::std::vector<float32> buffer;
} Texture;

class Material {
 public:
  virtual ~Material() {}
  // Returns the globally unique id for the material instance.
  uint32 GetID() { return material_id_; }
  // Returns true if the material is a light emitting material.
  virtual bool IsLight() { return false; }
  // Returns true if the material can potentially use transmitted light.
  // False otherwise, which indicates a fully opaque / diffuse material.
  virtual bool WillUseTransmittedLight() const = 0;
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  // Indirect includes both refracted and transmitted.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const = 0;
  // Returns a reflection vector based on the solid angle of the material.
  virtual vector3 Reflection(const vector3 &view, const vector3 &normal,
                             bool is_internal = false) const = 0;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  virtual vector3 Sample(float32 depth, const vector3 &sample_pos,
                         const vector3 &view_pos, const vector3 &view_dir,
                         const vector3 &light_pos, const vector3 &light_dir,
                         const vector3 &light_color,
                         const vector3 &surface_normal,
                         const vector2 &surface_texcoords = vector2(0, 0),
                         bool is_internal = false) = 0;

 protected:
  Material();
  // Unique material id.
  uint32 material_id_;
};

class DiffuseMaterial : public Material {
 public:
  DiffuseMaterial() :texture_scale_(0) {}
  DiffuseMaterial(const vector3 &diffuse);
  DiffuseMaterial(const ::std::string &filename, float32 tex_scale = 1.0f);
  virtual ~DiffuseMaterial() {}
  // Indicates that this material does not support transmitted light.
  virtual bool WillUseTransmittedLight() const override { return false; }
  // Loads a texture map into the diffuse channel of the material.
  void LoadDiffuseTexture(const ::std::string &filename,
                          float32 tex_scale = 1.0f);
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const override;
  // Returns a reflection vector based on the solid angle of the material.
  vector3 Reflection(const vector3 &view, const vector3 &normal,
                     bool is_internal = false) const override;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  vector3 Sample(float32 depth, const vector3 &sample_pos,
                 const vector3 &view_pos, const vector3 &view_dir,
                 const vector3 &light_pos, const vector3 &light_dir,
                 const vector3 &light_color, const vector3 &surface_normal,
                 const vector2 &surface_texcoords = vector2(0, 0),
                 bool is_internal = false) override;

 protected:
  // The diffuse component of reflected light.
  vector3 diffuse_;
  // Specifies a diffuse texture map to use in place of diffuse.
  Texture diffuse_map_;
  // Scaling factor applied to texture coordinates during sampling.
  // Only applied when diffuse_map_ is valid.
  float32 texture_scale_;
  // Samples the diffuse map if available. Returns the diffuse color if
  // no diffuse map has been loaded into the material.
  vector3 SampleDiffuse(const vector2 &texcoords);
};

class LightMaterial : public DiffuseMaterial {
 public:
  LightMaterial() : emissive_(1, 1, 1) { diffuse_ = vector3(1, 1, 1); };
  LightMaterial(const vector3 &emissive);
  virtual ~LightMaterial() {}
  // Returns true if the material is light emitting.
  virtual bool IsLight() { return true; }
  // Indicates that this material does not support transmitted light.
  virtual bool WillUseTransmittedLight() const override { return false; }
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const override;
  // Returns a reflection vector based on the solid angle of the material.
  virtual vector3 Reflection(const vector3 &view, const vector3 &normal,
                             bool is_internal = false) const override;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  virtual vector3 Sample(float32 depth, const vector3 &sample_pos,
                         const vector3 &view_pos, const vector3 &view_dir,
                         const vector3 &light_pos, const vector3 &light_dir,
                         const vector3 &light_color,
                         const vector3 &surface_normal,
                         const vector2 &surface_texcoords = vector2(0, 0),
                         bool is_internal = false) override;

 protected:
  // The ambient emission from the material (useful for light sources).
  vector3 emissive_;
};

class MetalMaterial : public DiffuseMaterial {
 public:
  MetalMaterial() : roughness_(0.5f) {}
  MetalMaterial(const vector3 &diffuse, float32 roughness);
  MetalMaterial(::std::string &filename, float32 roughness);
  // Indicates that this material does not support transmitted light.
  virtual bool WillUseTransmittedLight() const override { return false; }
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const override;
  vector3 Reflection(const vector3 &view, const vector3 &normal,
                     bool is_internal = false) const override;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  vector3 Sample(float32 depth, const vector3 &sample_pos,
                 const vector3 &view_pos, const vector3 &view_dir,
                 const vector3 &light_pos, const vector3 &light_dir,
                 const vector3 &light_color, const vector3 &surface_normal,
                 const vector2 &surface_texcoords = vector2(0, 0),
                 bool is_internal = false) override;

 protected:
  // Interpolation component between specular and diffuse values.
  float32 roughness_;
};

class MirrorMaterial : public DiffuseMaterial {
 public:
  MirrorMaterial() {}
  MirrorMaterial(const vector3 &diffuse);
  virtual ~MirrorMaterial() {}
  // Indicates that this material does not support transmitted light.
  virtual bool WillUseTransmittedLight() const override { return false; }
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const override;
  vector3 Reflection(const vector3 &view, const vector3 &normal,
                     bool is_internal = false) const override;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  vector3 Sample(float32 depth, const vector3 &sample_pos,
                 const vector3 &view_pos, const vector3 &view_dir,
                 const vector3 &light_pos, const vector3 &light_dir,
                 const vector3 &light_color, const vector3 &surface_normal,
                 const vector2 &surface_texcoords = vector2(0, 0),
                 bool is_internal = false) override;
};

class GlassMaterial : public DiffuseMaterial {
 public:
  GlassMaterial() {}
  GlassMaterial(const vector3 diffuse, float32 index = 0.75f, float32 reflectivity = 0.1, float32 frost = 0.0f);
  virtual ~GlassMaterial() {}
  // Indicates that this material does support transmitted light.
  virtual bool WillUseTransmittedLight() const override { return true; }
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const override;
  vector3 Reflection(const vector3 &view, const vector3 &normal,
                     bool is_internal = false) const override;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  vector3 Sample(float32 depth, const vector3 &sample_pos,
                 const vector3 &view_pos, const vector3 &view_dir,
                 const vector3 &light_pos, const vector3 &light_dir,
                 const vector3 &light_color, const vector3 &surface_normal,
                 const vector2 &surface_texcoords = vector2(0, 0),
                 bool is_internal = false) override;

protected:
    float32 index_;
    float32 frost_;
    float32 reflectivity_;
};

class LiquidMaterial : public DiffuseMaterial {
 public:
  LiquidMaterial() {}
  LiquidMaterial(const vector3 diffuse, float32 index = 0.75f, float32 reflectivity = 0.4f);
  virtual ~LiquidMaterial() {}
  // Indicates that this material does support transmitted light.
  virtual bool WillUseTransmittedLight() const override { return false; }
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const override;
  vector3 Reflection(const vector3 &view, const vector3 &normal,
                     bool is_internal = false) const override;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  vector3 Sample(float32 depth, const vector3 &sample_pos,
                 const vector3 &view_pos, const vector3 &view_dir,
                 const vector3 &light_pos, const vector3 &light_dir,
                 const vector3 &light_color, const vector3 &surface_normal,
                 const vector2 &surface_texcoords = vector2(0, 0),
                 bool is_internal = false) override;

protected:
    float32 index_;
    float32 reflectivity_;
};

class CeramicMaterial : public DiffuseMaterial {
 public:
  CeramicMaterial() : shininess_(0.0f) {}
  CeramicMaterial(const vector3 &diffuse, float32 shininess);
  // Indicates that this material does not support transmitted light.
  virtual bool WillUseTransmittedLight() const override { return false; }
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const override;
  vector3 Reflection(const vector3 &view, const vector3 &normal,
                     bool is_internal = false) const override;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  vector3 Sample(float32 depth, const vector3 &sample_pos,
                 const vector3 &view_pos, const vector3 &view_dir,
                 const vector3 &light_pos, const vector3 &light_dir,
                 const vector3 &light_color, const vector3 &surface_normal,
                 const vector2 &surface_texcoords = vector2(0, 0),
                 bool is_internal = false) override;

 protected:
  // Interpolation component between specular and diffuse values.
  float32 shininess_;
};

class GlowMaterial : public CeramicMaterial {
 public:
     GlowMaterial(const vector3 &diffuse, const vector3 &glow, float32 shininess);
     vector3 Sample(float32 depth, const vector3& sample_pos,
         const vector3& view_pos, const vector3& view_dir,
         const vector3& light_pos, const vector3& light_dir,
         const vector3& light_color, const vector3& surface_normal,
         const vector2& surface_texcoords = vector2(0, 0),
         bool is_internal = false) override;
protected:
    // A glow color to be emitted from the surface.
    vector3 glow_;
};

class FogMaterial : public DiffuseMaterial {
 public:
  FogMaterial() : density_(0) {}
  FogMaterial(const vector3 diffuse, float32 density);
  virtual ~FogMaterial() {}
  // Indicates that this material does support transmitted light.
  virtual bool WillUseTransmittedLight() const override { return true; }
  // Returns true if the material will use indirect light, given the incident
  // light vector and the object surface normal. Returns false otherwise.
  virtual bool WillUseIndirectLight(const vector3 &incident_light,
                                    const vector3 &normal) const override;
  vector3 Reflection(const vector3 &view, const vector3 &normal,
                     bool is_internal = false) const override;
  // Determines the color of reflected light according to the material
  // properties and the input parameters.
  virtual vector3 Sample(float32 depth, const vector3 &sample_pos,
                 const vector3 &view_pos, const vector3 &view_dir,
                 const vector3 &light_pos, const vector3 &light_dir,
                 const vector3 &light_color, const vector3 &surface_normal,
                 const vector2 &surface_texcoords = vector2(0, 0),
                 bool is_internal = false) override;

 protected:
  float32 density_;
};

void InitializeMaterials();

}  // namespace base

#endif  // __MATERIAL_H__