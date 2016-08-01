/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TANGOSCENERECONSTRUCTOR_SCENE_H_
#define TANGOSCENERECONSTRUCTOR_SCENE_H_

#include <tango_client_api.h>  // NOLINT
#include <tango-gl/axis.h>
#include <tango-gl/camera.h>
#include <tango-gl/color.h>
#include <tango-gl/gesture_camera.h>
#include <tango-gl/grid.h>

#include "tango-scene-reconstructor/point_cloud_drawable.h"

namespace tango_scene_reconstructor {

  class Scene {
    public:
      Scene();
      ~Scene();
      void FreeGLContent();
      void SetViewPort(int w, int h);
      void Reset();
      void Render(const glm::mat4& tango_pose,
                  const glm::mat4& point_cloud_transformation,
                  const std::vector<float>& point_cloud_data,
                  const std::vector<uint8_t>& rgb_data);
      void SetCameraType(tango_gl::GestureCamera::CameraType camera_type);
      void OnTouchEvent(int touch_count, tango_gl::GestureCamera::TouchEvent event,
                        float x0, float y0, float x1, float y1);
      void SetBackgroundColor(bool on);
      void SetGridOn(bool on);

    private:
      tango_gl::GestureCamera* gesture_camera_;
      tango_gl::Axis* axis_;
      tango_gl::Grid* grid_;
      PointCloudDrawable* pcd_drawable_;
      glm::vec3 kHeightOffset_;
      bool backgroundColorBlack_;
      bool draw_grid_;
  };

} // namespace tango_scene_reconstructor

#endif  // TANGOSCENERECONSTRUCTOR_SCENE_H_
