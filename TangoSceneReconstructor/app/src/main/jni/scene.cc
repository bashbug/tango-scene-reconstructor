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

#include "tango-scene-reconstructor/scene.h"

namespace tango_scene_reconstructor {

  Scene::Scene() {
    kHeightOffset_ = glm::vec3(0.0f, 1.3f, 0.0f);
    const tango_gl::Color kGridColor_(0.2f, 0.2f, 0.2f);
    gesture_camera_ = new tango_gl::GestureCamera();
    axis_ = new tango_gl::Axis();
    grid_ = new tango_gl::Grid();
    pcd_drawable_ = new tango_scene_reconstructor::PointCloudDrawable();

    grid_->SetColor(kGridColor_);
    grid_->SetPosition(-kHeightOffset_);
    gesture_camera_->SetCameraType(tango_gl::GestureCamera::CameraType::kFirstPerson);
    backgroundColorBlack_ = true;
    draw_grid_ = true;
  }

  Scene::~Scene() {}

  void Scene::FreeGLContent() {
    delete gesture_camera_;
    delete axis_;
    delete grid_;
    delete pcd_drawable_;
  }

  void Scene::SetViewPort(int w, int h) {

    LOGE("width %i, height %i", w, h);
    if (h == 0) {
      LOGE("Setup graphic height not valid");
    }
    gesture_camera_->SetAspectRatio(static_cast<float>(w) /
                                    static_cast<float>(h));
    glViewport(0, 0, w, h);
  }

  void Scene::Reset() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  }

  void Scene::Render(const glm::mat4& tango_pose,
                     const glm::mat4& point_cloud_transformation,
                     const std::vector<float>& point_cloud_data,
                     const std::vector<uint8_t>& rgb_data) {

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    if (backgroundColorBlack_) {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    } else {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    }
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    glm::vec3 position =
        glm::vec3(tango_pose[3][0], tango_pose[3][1],
                  tango_pose[3][2]);

    if (gesture_camera_->GetCameraType() ==
        tango_gl::GestureCamera::CameraType::kFirstPerson) {
      // In first person mode, we directly control camera's motion.
      gesture_camera_->SetTransformationMatrix(tango_pose);
    } else {
      // In third person or top down more, we follow the camera movement.
      gesture_camera_->SetAnchorPosition(position);
      if (draw_grid_) {
        grid_->SetPosition(kHeightOffset_);
      }
      axis_->SetTransformationMatrix(tango_pose);
      axis_->Render(gesture_camera_->GetProjectionMatrix(),
                    gesture_camera_->GetViewMatrix());
    }

    if (draw_grid_) {
      grid_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());
    }

    if (point_cloud_data.size() > 0 && rgb_data.size() > 0) {
      pcd_drawable_->Render(gesture_camera_->GetProjectionMatrix(),
                            gesture_camera_->GetViewMatrix(),
                            point_cloud_transformation,
                            point_cloud_data,
                            rgb_data);
    }
  }

  void Scene::SetCameraType(tango_gl::GestureCamera::CameraType camera_type) {
    gesture_camera_->SetCameraType(camera_type);
  }

  void Scene::OnTouchEvent(int touch_count,
                           tango_gl::GestureCamera::TouchEvent event, float x0,
                           float y0, float x1, float y1) {
    gesture_camera_->OnTouchEvent(touch_count, event, x0, y0, x1, y1);
  }

  void Scene::SetBackgroundColor(bool on) {
    backgroundColorBlack_ = on;
  }

  void Scene::SetGridOn(bool on) {
    draw_grid_ = on;
  }

}  // namespace rgb_depth_sync
