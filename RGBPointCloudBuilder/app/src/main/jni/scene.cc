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

#include "rgb-depth-sync/scene.h"
#include "rgb-depth-sync/pcd_drawable.h"
#include "rgb-depth-sync/shader.h"


namespace {
// We want to represent the device properly with respect to the ground so we'll
// add an offset in z to our origin. We'll set this offset to 1.3 meters based
// on the average height of a human standing with a Tango device. This allows us
// to place a grid roughly on the ground for most users.
  glm::vec3 kHeightOffset = glm::vec3(0.0f, 1.3f, 0.0f);
  //glm::vec3 kHeightOffset_neg = glm::vec3(0.0f, 0.0f, 0.0f);
// Color of the motion tracking trajectory.
  const tango_gl::Color kTraceColor(0.22f, 0.28f, 0.67f);

  const tango_gl::Color kTraceColorICP(0.0f, 1.0f, 0.0f);

// Color of the ground grid.
  const tango_gl::Color kGridColor(0.2f, 0.2f, 0.2f);

// Frustum scale.
  const glm::vec3 kFrustumScale = 2.0f*glm::vec3(0.4f, 0.3f, 0.5f);

  const float kCubeScale = 1.5f;

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
    cube_ = new tango_gl::Cube();
    cube_->render_mode_ = GL_LINES;
    cube_->SetColor(0.7f, 0.7f, 0.7f);
    /*texture_ = new rgb_depth_sync::TextureDrawable(shader::kPointCloudVertex,
                                                   shader::kPointCloudFragment);*/
    pcd_drawable_ = new rgb_depth_sync::PCDDrawable();

    first_ = true;

    trace_->SetColor(kTraceColor);
    trace_icp_->SetColor(kTraceColorICP);
    grid_->SetColor(kGridColor);
    frustum_->SetColor(kGridColor);
    grid_->SetPosition(-kHeightOffset);
    gesture_camera_->SetCameraType(tango_gl::GestureCamera::CameraType::kFirstPerson);
  }

  Scene::~Scene() {}

  void Scene::FreeGLContent() {
    delete gesture_camera_;
    delete axis_;
    delete frustum_;
    delete trace_;
    delete trace_icp_;
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

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
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
      //grid_->SetPosition(kHeightOffset_neg);

      /*frustum_->SetTransformationMatrix(tango_pose);
      // Set the frustum scale to 4:3, this doesn't necessarily match the physical
      // camera's aspect ratio, this is just for visualization purposes.
      frustum_->SetScale(kFrustumScale);
      frustum_->Render(gesture_camera_->GetProjectionMatrix(),
                       gesture_camera_->GetViewMatrix());*/

      axis_->SetTransformationMatrix(tango_pose);
      axis_->Render(gesture_camera_->GetProjectionMatrix(),
                    gesture_camera_->GetViewMatrix());
    }

    //trace_->UpdateVertexArray(position);
    /*trace_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());

    //trace_icp_->UpdateVertexArray(position_icp);
    trace_icp_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());
    grid_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());*/

    if (point_cloud_data.size() > 0 && rgb_data.size() > 0) {
      pcd_drawable_->Render(gesture_camera_->GetProjectionMatrix(),
                            gesture_camera_->GetViewMatrix(),
                            point_cloud_transformation,
                            point_cloud_data,
                            rgb_data);
    }
    /*if (first_) {
      glm::mat4 mvp_mat = tango_pose;
      //cube_->SetInitPosition(glm::inverse(tango_pose));
      cube_->SetTransformationMatrix(glm::translate(glm::mat4(1.0f), glm::vec3(0.0, -0.3, -0.6f)));
      first_ = false;
    }

    cube_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());*/

    //glDisable(GL_DEPTH_TEST);
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
