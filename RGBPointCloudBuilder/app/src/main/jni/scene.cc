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

#include <tango-gl/conversions.h>

#include "rgb-depth-sync/scene.h"
#include "rgb-depth-sync/texture_drawable.h"
#include "rgb-depth-sync/shader.h"


namespace {
// We want to represent the device properly with respect to the ground so we'll
// add an offset in z to our origin. We'll set this offset to 1.3 meters based
// on the average height of a human standing with a Tango device. This allows us
// to place a grid roughly on the ground for most users.
  const glm::vec3 kHeightOffset = glm::vec3(0.0f, 1.3f, 0.0f);

// Color of the motion tracking trajectory.
  const tango_gl::Color kTraceColor(0.22f, 0.28f, 0.67f);

  const tango_gl::Color kTraceColorICP(0.0f, 1.0f, 0.0f);

// Color of the ground grid.
  const tango_gl::Color kGridColor(0.85f, 0.85f, 0.85f);

// Frustum scale.
  const glm::vec3 kFrustumScale = glm::vec3(1.0f, 1.0f, 1.0f);

  static const glm::mat4 kDepth_T_OpenGL =
      glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, -1.0f);
}  // namespace

namespace rgb_depth_sync {

  Scene::Scene() {
    gesture_camera_ = new tango_gl::GestureCamera();
    axis_ = new tango_gl::Axis();
    frustum_ = new tango_gl::Frustum();
    trace_ = new tango_gl::Trace();
    trace_icp_ = new tango_gl::Trace();
    grid_ = new tango_gl::Grid();
    texture_ = new rgb_depth_sync::TextureDrawable(shader::kPointCloudVertex,
                                                   shader::kPointCloudFragment);

    trace_->SetColor(kTraceColor);
    trace_icp_->SetColor(kTraceColorICP);
    grid_->SetColor(kGridColor);
    grid_->SetPosition(-kHeightOffset);
    gesture_camera_->SetCameraType(tango_gl::GestureCamera::CameraType::kThirdPerson);
  }

  Scene::~Scene() {}

  void Scene::FreeGLContent() {
    delete gesture_camera_;
    delete axis_;
    delete frustum_;
    delete trace_;
    delete trace_icp_;
    delete grid_;
    delete texture_;
  }

  void Scene::SetViewPort(int w, int h) {
    if (h == 0) {
      LOGE("Setup graphic height not valid");
    }
    gesture_camera_->SetAspectRatio(static_cast<float>(w) /
                                    static_cast<float>(h));
    glViewport(0, 0, w, h);
  }

  void Scene::Render(const glm::mat4& tango_pose,
                     const glm::mat4& point_cloud_transformation,
                     const glm::mat4& icp_pose,
                     const std::vector<float>& point_cloud_data,
                     const std::vector<uint8_t>& rgb_data) {

    glEnable(GL_DEPTH_TEST);
    tango_gl::util::CheckGlError("PointCloud glEnable(GL_DEPTH_TEST)");
    glEnable(GL_CULL_FACE);
    tango_gl::util::CheckGlError("PointCloud glEnable(GL_CULL_FACE)");
    glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    tango_gl::util::CheckGlError("PointCloud glClearColor(1.0f, 1.0f, 1.0f, 1.0f)");
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    tango_gl::util::CheckGlError("PointCloud glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)");

    glm::vec3 position = glm::vec3(tango_pose[3][0], tango_pose[3][1], tango_pose[3][2]);

    glm::vec3 position_icp = glm::vec3(icp_pose[3][0], icp_pose[3][1], icp_pose[3][2]);

    //LOGE("icp pose: x:%f y:%f z:%f", position_icp[0], position_icp[1], position_icp[2]);

    if (gesture_camera_->GetCameraType() ==
        tango_gl::GestureCamera::CameraType::kFirstPerson) {
      // In first person mode, we directly control camera's motion.Render
      gesture_camera_->SetTransformationMatrix(tango_pose);
    } else {
      // In third person or top down more, we follow the camera movement.
      gesture_camera_->SetAnchorPosition(position);

      frustum_->SetTransformationMatrix(tango_pose);
      // Set the frustum scale to 4:3, this doesn't necessarily match the physical
      // camera's aspect ratio, this is just for visualization purposes.
      frustum_->SetScale(kFrustumScale);
      frustum_->Render(gesture_camera_->GetProjectionMatrix(),
                       gesture_camera_->GetViewMatrix());

      axis_->SetTransformationMatrix(tango_pose);
      axis_->Render(gesture_camera_->GetProjectionMatrix(),
                    gesture_camera_->GetViewMatrix());
    }

    //trace_->UpdateVertexArray(position);
    trace_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());

    //trace_icp_->UpdateVertexArray(position_icp);
    trace_icp_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());

    grid_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());

    texture_->RenderPointCloud(gesture_camera_->GetProjectionMatrix(),
                               gesture_camera_->GetViewMatrix(),
                               point_cloud_transformation, point_cloud_data,
                               rgb_data);
  }

  void Scene::SetTrace(std::vector<glm::vec3> positions) {

    for (int i=0; i<positions.size(); i++) {
      trace_->UpdateVertexArray(positions[i]);
    }
  }

  void Scene::SetICPTrace(std::vector<glm::vec3> positions) {
    for (int i=0; i<positions.size(); i++) {
      trace_icp_->UpdateVertexArray(positions[i]);
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

}  // namespace rgb_depth_sync
