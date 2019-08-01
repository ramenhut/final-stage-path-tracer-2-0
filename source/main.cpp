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

void PrintUsage(const char *programName) {
  printf("Usage: %s [options]\n", programName);
  printf("  --file [scene filename]  \tSpecifies the scene file to load.\n");
  printf("  --width [integer]  \t\tSets the width of the output frame.\n");
  printf("  --height [integer]  \t\tSets the height of the output frame.\n");
}

int main(int argc, char **argv) {
  printf(
      "Copyright (c) 2006-2019 Joe Bertolami. All Right Reserved.\nFor more "
      "information visit https://bertolami.com.\n\n");

  ::std::string scene_filename;
  ::base::uint32 window_width = 800;
  ::base::uint32 window_height = 480;

  if (argc <= 1) {
    PrintUsage(argv[0]);
    return 0;
  }

  for (int i = 1; i < argc; i++) {
    char *optBegin = argv[i];
    for (int j = 0; j < 2; j++) (optBegin[0] == '-') ? optBegin++ : optBegin;

    switch (optBegin[0]) {
      case 'f':
        scene_filename = argv[++i];
        break;
      case 'w':
        window_width = atoi(argv[++i]);
        break;
      case 'h':
        window_height = atoi(argv[++i]);
        break;
    }
  }

  if (!scene_filename.length()) {
    printf("You must specify a scene filename (-f filename).\n");
    return 0;
  }

  printf("Loading scene %s and rendering at %ix%i resolution.\n",
         scene_filename.c_str(), window_width, window_height);

  ::base::Scene scene;
  ::base::Camera camera(::base::vector3(-5.80, 7.05, -47.06),
                        ::base::vector3(0.00, 8.94, 0.00));

  if (!scene.LoadScene(scene_filename)) {
    return 0;
  }

  ::std::unique_ptr<::base::GraphicsWindow> window =
      ::std::make_unique<::base::GraphicsWindow>(
          "Final Stage Path Tracer 2.02", 100, 10, window_width,
          window_height, 32, 0);
  ::std::vector<::base::InputEvent> window_events;
  ::base::ImagePlaneCache image_cache(window_width, window_height);
  ::base::DisplayFrame output_frame(window_width, window_height);

  ::base::InitializeMaterials();

  if (scene.GetCameraCount()) {
    camera = *scene.GetCamera(0);
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
        printf("Origin: %.2f, %.2f, %.2f\n", camera.origin.x, camera.origin.y,
               camera.origin.z);
        output_frame.Reset();
        image_cache.Invalidate();
      } else if (event.switch_index == ::base::kInputMouseRightButtonIndex &&
                 event.is_on) {
        camera.focal_depth = ::base::TraceRange(
            camera, &scene, &output_frame,
            (event.target_x + 1) * output_frame.GetWidth() * 0.5f,
            (event.target_y + 1) * output_frame.GetHeight() * 0.5f);

        printf("Focus: %.2f\n", camera.focal_depth);
        output_frame.Reset();
        image_cache.Invalidate();
      }
    }

    ::base::TraceScene(camera, &scene, &output_frame);

    window->BeginScene();
    glClearColor(0.5f, 0.5f, 0.4f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawPixels(window_width, window_height, GL_RGB, GL_UNSIGNED_BYTE,
                 output_frame.GetDisplayBuffer());
    window->EndScene();
  }

  return 0;
}