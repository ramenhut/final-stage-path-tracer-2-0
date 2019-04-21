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

#include "engine.h"
#include "frame.h"
#include "math/intersect.h"
#include "math/random.h"
#include "stdio.h"
#include "stdlib.h"
#include "window/base_graphics.h"

const ::base::uint32 kWidthWidth = 800;
const ::base::uint32 kWindowHeight = 480;

int main(int argc, char **argv) {
  printf("Copyright (c) 2006-2018 Joe Bertolami. All Right Reserved.\n");
  ::std::unique_ptr<::base::GraphicsWindow> window =
      ::std::make_unique<::base::GraphicsWindow>("Final Stage Path Tracer 2.0",
                                                 100, 10, kWidthWidth,
                                                 kWindowHeight, 32, 0);
  ::std::vector<::base::InputEvent> window_events;
  ::base::ImagePlaneCache image_cache(kWidthWidth, kWindowHeight);
  ::base::DisplayFrame output_frame(kWidthWidth, kWindowHeight);
  ::base::Scene scene;
  ::base::Camera camera(::base::vector3(-9.80, 9.05, -87.06),
                        ::base::vector3(0.00, 8.94, 0.00));

  ::base::InitializeMaterials();

  // Stone floor.
  auto white_diffuse_mat = ::std::make_shared<::base::DiffuseMaterial>(
      ::base::vector3(0.6f, 0.6f, 0.6f));
  white_diffuse_mat->LoadDiffuseTexture("assets/textures/stone3.bmp", 0.005f);
  // HDR background.
  auto hdr_mat =
      ::std::make_shared<::base::LightMaterial>(::base::vector3(5, 5, 5));
  hdr_mat->LoadDiffuseTexture("assets/backgrounds/knoll.exr");

  // Option 1: Clay (fully diffuse)
  auto clay_mat = ::std::make_shared<::base::DiffuseMaterial>(
      ::base::vector3(0.4f, 0.4f, 0.6f));
  // Option 2: Ceramic (diffuse + specular)
  auto ceramic_mat = ::std::make_shared<::base::CeramicMaterial>(
      ::base::vector3(1.0, 0.7, 0.4), 0.2f);
  // Option 3: Metallic (constrained reflection)
  auto metal_mat = ::std::make_shared<::base::MetalMaterial>(
      ::base::vector3(0.9, 0.7, 0.4f), 0.2f);
  // Option 4: Paper (constrained reflection)
  auto paper_mat = ::std::make_shared<::base::MetalMaterial>(
      ::base::vector3(0.7, 0.7, 0.4f), 0.4f);
  // Option 5: Glass (reflection + refraction)
  auto glass_mat = ::std::make_shared<::base::GlassMaterial>(
      ::base::vector3(0.95, 0.65, 0.45));
  // Option 6: Liquid (reflection + refraction)
  auto liquid_mat = ::std::make_shared<::base::LiquidMaterial>(
      ::base::vector3(0.85, 0.75, 0.95));
  // Option 7: Mirror (perfectly reflective)
  auto mirror_mat =
      ::std::make_shared<::base::MirrorMaterial>(::base::vector3(1, 1, 1));
  // Option 8: Magical (emissive + ceramic)
  auto magical_mat = ::std::make_shared<::base::GlowMaterial>(
      ::base::vector3(1.0, 0.7, 0.8), ::base::vector3(0.4, 0.1, 0.4), 0.2f);
  // Option 9: White light (full emission)
  auto light_mat =
      ::std::make_shared<::base::LightMaterial>(::base::vector3(20, 20, 20));

  scene.SetSkyMaterial(hdr_mat);

  // Stanford dragon (primary object)
  ::base::Object *mesh_object =
      scene.AddMeshObject("assets/meshes/dragon.obj", true, ::base::vector3(0, -10, 0),
                          ::base::vector3(1, 1, 1));
  mesh_object->SetMaterial(metal_mat);

  {  // floor
    ::base::Object *plane_object =
        scene.AddPlanarObject(::base::plane(0, 1, 0, 10));
    plane_object->SetMaterial(white_diffuse_mat);
  }

  bool mouse_down = false;
  ::base::float32 last_x = 0.0f;
  ::base::float32 last_y = 0.0f;
  ::base::float32 x_delta = 0.0f;
  ::base::float32 y_delta = 0.0f;

  while (window && window->IsValid()) {
    window->Update(&window_events);
    for (auto &event : window_events) {
      if (event.switch_index == 27) {
        return 0;
      }
      if (event.switch_index == ::base::kInputMouseLeftButtonIndex) {
        last_x = event.target_x;
        last_y = event.target_y;
        mouse_down = event.is_on;
        camera.fast_render_enabled = mouse_down;
        output_frame.Reset();
        image_cache.Invalidate();
      } else if (event.switch_index == ::base::kInputMouseMoveIndex &&
                 mouse_down) {
        x_delta = event.target_x - last_x;
        y_delta = event.target_y - last_y;
        last_x = event.target_x;
        last_y = event.target_y;

        camera.origin =
            camera.origin.rotate(x_delta * 3.0f, ::base::vector3(0, 1, 0));
        ::base::vector3 right_vec = (camera.target - camera.origin)
                                        .normalize()
                                        .cross(::base::vector3(0, 1, 0));
        camera.origin = camera.origin.rotate(y_delta * 3.0f, right_vec);
        output_frame.Reset();
        image_cache.Invalidate();
      } else if (event.switch_index == ::base::kInputMouseRightButtonIndex &&
                 event.is_on) {
        camera.focal_depth = ::base::TraceRange(
            camera, &scene, &output_frame,
            (event.target_x + 1) * output_frame.GetWidth() * 0.5f,
            (event.target_y + 1) * output_frame.GetHeight() * 0.5f);
        output_frame.Reset();
        image_cache.Invalidate();
      } else if (event.switch_index >= 49 && event.switch_index <= 57 &&
                 event.is_on) {
        switch (event.switch_index) {
          case 49:
            printf("Selected CLAY material.\n");
            mesh_object->SetMaterial(clay_mat);
            break;
          case 50:
            printf("Selected CERAMIC material.\n");
            mesh_object->SetMaterial(ceramic_mat);
            break;
          case 51:
            printf("Selected METAL material.\n");
            mesh_object->SetMaterial(metal_mat);
            break;
          case 52:
            printf("Selected PAPER material.\n");
            mesh_object->SetMaterial(paper_mat);
            break;
          case 53:
            printf("Selected GLASS material.\n");
            mesh_object->SetMaterial(glass_mat);
            break;
          case 54:
            printf("Selected LIQUID material.\n");
            mesh_object->SetMaterial(liquid_mat);
            break;
          case 55:
            printf("Selected MIRROR material.\n");
            mesh_object->SetMaterial(mirror_mat);
            break;
          case 56:
            printf("Selected MAGICAL material.\n");
            mesh_object->SetMaterial(magical_mat);
            break;
          case 57:
            printf("Selected LIGHT material.\n");
            mesh_object->SetMaterial(light_mat);
            break;
        }
        output_frame.Reset();
        image_cache.Invalidate();
      }
    }

    // One pass over the scene using the path tracer.
    ::base::TraceScene(camera, &scene, &output_frame);

    window->BeginScene();
    glClearColor(0.5f, 0.5f, 0.4f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawPixels(kWidthWidth, kWindowHeight, GL_RGB, GL_UNSIGNED_BYTE,
                 output_frame.GetDisplayBuffer());
    window->EndScene();
  }

  return 0;
}