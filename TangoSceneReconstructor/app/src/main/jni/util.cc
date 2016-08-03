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

#include "tango-scene-reconstructor/util.h"

namespace tango_scene_reconstructor {

glm::mat4 util::GetMatrixFromPose(const TangoPoseData* pose_data) {
  glm::vec3 translation =
      glm::vec3(pose_data->translation[0], pose_data->translation[1],
                pose_data->translation[2]);
  glm::quat rotation =
      glm::quat(pose_data->orientation[3], pose_data->orientation[0],
                pose_data->orientation[1], pose_data->orientation[2]);
  return glm::translate(glm::mat4(1.0f), translation) *
         glm::mat4_cast(rotation);
}

glm::quat util::GetRotationFromMatrix(glm::mat4 transformation) {
  glm::vec3 translation;
  glm::quat rotation;
  glm::vec3 scale;

  tango_gl::util::DecomposeMatrix(transformation, translation, rotation, scale);

  return  rotation;
}

glm::vec3 util::GetTranslationFromMatrix(glm::mat4 transformation) {
  glm::vec3 translation;
  glm::quat rotation;
  glm::vec3 scale;

  tango_gl::util::DecomposeMatrix(transformation, translation, rotation, scale);

  return translation;
}

glm::mat4 util::GetPoseAppliedOpenGLWorldFrame( const glm::mat4 pose_matrix) {
  // This full multiplication is equal to:
  //   opengl_world_T_opengl_camera =
  //      opengl_world_T_start_service *
  //      start_service_T_device *
  //      device_T_imu *
  //      imu_T_color_camera *
  //      depth_camera_T_opengl_camera;
  //
  // More information about frame transformation can be found here:
  // Frame of reference:
  //   https://developers.google.com/project-tango/overview/frames-of-reference
  // Coordinate System Conventions:
  //   https://developers.google.com/project-tango/overview/coordinate-systems
  return tango_gl::conversions::opengl_world_T_tango_world()
         * pose_matrix;
}

glm::mat4 util::ConvertEigenToGLMPose(const Eigen::Isometry3f eigen_pose) {
  // glm;;mat[] returns a column vector

  glm::mat4 pose;
  //Rotation
  pose[0][0] = eigen_pose(0,0);
  pose[0][1] = eigen_pose(1,0);
  pose[0][2] = eigen_pose(2,0);
  pose[0][3] = eigen_pose(3,0);

  pose[1][0] = eigen_pose(0,1);
  pose[1][1] = eigen_pose(1,1);
  pose[1][2] = eigen_pose(2,1);
  pose[1][3] = eigen_pose(3,1);

  pose[2][0] = eigen_pose(0,2);
  pose[2][1] = eigen_pose(1,2);
  pose[2][2] = eigen_pose(2,2);
  pose[2][3] = eigen_pose(3,2);

  //Translation
  pose[3][0] = eigen_pose(0,3);
  pose[3][1] = eigen_pose(1,3);
  pose[3][2] = eigen_pose(2,3);
  pose[3][3] = eigen_pose(3,3);
  return pose;
}

Eigen::Isometry3f util::ConvertGLMToEigenPose(const glm::mat4 glm_pose) {
  // Eigen::Isometry3f(row, column)
  Eigen::Isometry3f pose;
  // Rotation
  pose(0,0) = glm_pose[0][0];
  pose(1,0) = glm_pose[0][1];
  pose(2,0) = glm_pose[0][2];
  pose(3,0) = glm_pose[0][3];

  pose(0,1) = glm_pose[1][0];
  pose(1,1) = glm_pose[1][1];
  pose(2,1) = glm_pose[1][2];
  pose(3,1) = glm_pose[1][3];

  pose(0,2) = glm_pose[2][0];
  pose(1,2) = glm_pose[2][1];
  pose(2,2) = glm_pose[2][2];
  pose(3,2) = glm_pose[2][3];

  // Translation
  pose(0,3) = glm_pose[3][0];
  pose(1,3) = glm_pose[3][1];
  pose(2,3) = glm_pose[3][2];
  pose(3,3) = glm_pose[3][3];
  return pose;
}

Eigen::Isometry3d util::CastIsometry3fTo3d(const Eigen::Isometry3f pose_f) {
  Eigen::Isometry3d pose_d;
  pose_d(0,0) = static_cast<double>(pose_f(0,0));
  pose_d(1,0) = static_cast<double>(pose_f(1,0));
  pose_d(2,0) = static_cast<double>(pose_f(2,0));
  pose_d(3,0) = static_cast<double>(pose_f(3,0));

  pose_d(0,1) = static_cast<double>(pose_f(0,1));
  pose_d(1,1) = static_cast<double>(pose_f(1,1));
  pose_d(2,1) = static_cast<double>(pose_f(2,1));
  pose_d(3,1) = static_cast<double>(pose_f(3,1));

  pose_d(0,2) = static_cast<double>(pose_f(0,2));
  pose_d(1,2) = static_cast<double>(pose_f(1,2));
  pose_d(2,2) = static_cast<double>(pose_f(2,2));
  pose_d(3,2) = static_cast<double>(pose_f(3,2));

  pose_d(0,3) = static_cast<double>(pose_f(0,3));
  pose_d(1,3) = static_cast<double>(pose_f(1,3));
  pose_d(2,3) = static_cast<double>(pose_f(2,3));
  pose_d(3,3) = static_cast<double>(pose_f(3,3));
  return pose_d;
}

Eigen::Isometry3f util::CastIsometry3dTo3f(const Eigen::Isometry3d pose_d) {
  Eigen::Isometry3f pose_f;
  pose_f(0,0) = static_cast<float>(pose_d(0,0));
  pose_f(1,0) = static_cast<float>(pose_d(1,0));
  pose_f(2,0) = static_cast<float>(pose_d(2,0));
  pose_f(3,0) = static_cast<float>(pose_d(3,0));

  pose_f(0,1) = static_cast<float>(pose_d(0,1));
  pose_f(1,1) = static_cast<float>(pose_d(1,1));
  pose_f(2,1) = static_cast<float>(pose_d(2,1));
  pose_f(3,1) = static_cast<float>(pose_d(3,1));

  pose_f(0,2) = static_cast<float>(pose_d(0,2));
  pose_f(1,2) = static_cast<float>(pose_d(1,2));
  pose_f(2,2) = static_cast<float>(pose_d(2,2));
  pose_f(3,2) = static_cast<float>(pose_d(3,2));

  pose_f(0,3) = static_cast<float>(pose_d(0,3));
  pose_f(1,3) = static_cast<float>(pose_d(1,3));
  pose_f(2,3) = static_cast<float>(pose_d(2,3));
  pose_f(3,3) = static_cast<float>(pose_d(3,3));
  return pose_f;
}

Eigen::Isometry3d util::CastGLMToEigenPosed(const glm::mat4& glm_pose) {
  // Eigen::Isometry3d(row, column)
  Eigen::Isometry3d pose;
  // Rotation
  pose(0,0) = static_cast<double>(glm_pose[0][0]);
  pose(1,0) = static_cast<double>(glm_pose[0][1]);
  pose(2,0) = static_cast<double>(glm_pose[0][2]);
  pose(3,0) = static_cast<double>(glm_pose[0][3]);

  pose(0,1) = static_cast<double>(glm_pose[1][0]);
  pose(1,1) = static_cast<double>(glm_pose[1][1]);
  pose(2,1) = static_cast<double>(glm_pose[1][2]);
  pose(3,1) = static_cast<double>(glm_pose[1][3]);

  pose(0,2) = static_cast<double>(glm_pose[2][0]);
  pose(1,2) = static_cast<double>(glm_pose[2][1]);
  pose(2,2) = static_cast<double>(glm_pose[2][2]);
  pose(3,2) = static_cast<double>(glm_pose[2][3]);

  // Translation
  pose(0,3) = static_cast<double>(glm_pose[3][0]);
  pose(1,3) = static_cast<double>(glm_pose[3][1]);
  pose(2,3) = static_cast<double>(glm_pose[3][2]);
  pose(3,3) = static_cast<double>(glm_pose[3][3]);
  return pose;
}

float util::Deg2Rad(float alpha) {
  return alpha*M_PI/180.0f;
}

Eigen::Isometry3f util::GetIsometryFromTangoPose(const TangoPoseData* pose_data) {
  Eigen::Vector3f translation(pose_data->translation[0], pose_data->translation[1], pose_data->translation[2]);
  // Eigen::Quaternionf(w, x, y, z) TangoPoseData orientation[] has x, y, z, w
  Eigen::Quaternionf rotation(pose_data->orientation[3], pose_data->orientation[0], pose_data->orientation[1], pose_data->orientation[2]);
  // normalize rotation before using it for any computation
  rotation.normalize();
  return Eigen::Isometry3f(Eigen::Translation3f(translation[0], translation[1], translation[2]))
         * Eigen::Isometry3f(rotation);
}

Eigen::Vector3f util::GetTranslationFromIsometry(const Eigen::Isometry3f& pose) {
  Eigen::Vector3f translation(pose.translation());
  return translation;
}

Eigen::Quaternionf util::GetRotationFromIsometry(const Eigen::Isometry3f& pose) {
  Eigen::Quaternionf rotation(pose.rotation());
  // normalize rotation before using it for any computation
  rotation.normalize();
  return rotation;
}

}
