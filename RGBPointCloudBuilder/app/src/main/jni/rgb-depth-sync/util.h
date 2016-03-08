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

#ifndef RGB_DEPTH_SYNC_UTIL_H_
#define RGB_DEPTH_SYNC_UTIL_H_

#include <Eigen/Geometry>

#include "tango_client_api.h"  // NOLINT
#include "tango-gl/util.h"

namespace rgb_depth_sync {
  // Utility functioins for Synchronization application.
  namespace util {
    Eigen::Isometry3f GetIsometryFromTangoPose(const TangoPoseData* pose_data);
    Eigen::Vector3f GetTranslationFromIsometry(const Eigen::Isometry3f& pose);
    Eigen::Quaternionf GetRotationFromIsometry(const Eigen::Isometry3f& pose);
    // Returns a transformation matrix from a given TangoPoseData structure.
    // - pose_data: The original pose is used for the conversion.
    glm::mat4 GetMatrixFromPose(const TangoPoseData* pose_data);
    glm::vec3 GetTranslationFromMatrix(const glm::mat4 transformation);
    glm::quat GetRotationFromMatrix(const glm::mat4 transformation);
    glm::mat4 GetPoseAppliedOpenGLWorldFrame( const glm::mat4 pose_matrix);
    Eigen::Isometry3d CastIsometry3fTo3d(const Eigen::Isometry3f pose_f);
    Eigen::Isometry3f CastIsometry3dTo3f(const Eigen::Isometry3d pose_d);
    Eigen::Isometry3d CastGLMToEigenPosed(const glm::mat4& glm_pose);
    Eigen::Isometry3f ConvertGLMToEigenPose(const glm::mat4 glm_pose);
    glm::mat4 ConvertEigenToGLMPose(const Eigen::Isometry3f eigen_pose);
    float Deg2Rad(float alpha);
    }  // namespace util
  }  // namespace rgb_depth_sync

#endif  // RGB_DEPTH_SYNC_UTIL_H_
