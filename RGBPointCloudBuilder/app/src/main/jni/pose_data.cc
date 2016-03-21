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

#include <sstream>
#include <tango-gl/conversions.h>

#include "rgb-depth-sync/pose_data.h"

namespace {
  const float kMeterToMillimeter = 1000.0f;
}  // namespace

namespace rgb_depth_sync {

  PoseData* PoseData::GetInstance() {
    static PoseData* instance = new rgb_depth_sync::PoseData();
    return instance;
  }

  void PoseData::SetImuTDevice(const glm::mat4& imu_T_device) {
    imu_T_device_ = imu_T_device;
  }

  void PoseData::SetImuTDepthCamera(const glm::mat4& imu_T_depth_camera) {
    imu_T_depth_camera_ = imu_T_depth_camera;
  }

  void PoseData::SetImuTColorCamera(const glm::mat4& imu_T_color) {
    imu_T_color_camera_ = imu_T_color;
  }

  void PoseData::SetDeviceTColorCamera(const glm::mat4& device_T_color) {
    device_T_color_camera_ = device_T_color;
  }

  void PoseData::SetDeviceTDepthCamera(const glm::mat4& device_T_depth) {
    device_T_depth_camera_ = device_T_depth;
  }

  void PoseData::SetColorCameraTDevice(const glm::mat4& color_T_device) {
    color_camera_T_device_ = color_T_device;
  }

  void PoseData::UpdatePose(const TangoPoseData* pose_data) {

    if(pose_data->status_code == TANGO_POSE_VALID) {
      cur_pose_ = *pose_data;
    }

    if (prev_pose_.status_code != cur_pose_.status_code) {
      // Reset pose counter when the status changed.
      pose_counter_ = 0;
    }

    // Increase pose counter.
    ++pose_counter_;
    FormatPoseString();
    prev_pose_ = cur_pose_;
  }

  std::string PoseData::GetPoseDebugString() {
    return pose_string_;
  }

  glm::mat4 PoseData::GetLatestPoseMatrix() {
    return GetMatrixFromPose(cur_pose_);
  }

  glm::mat4 PoseData::GetPoseAtTime(double timestamp) {
    TangoPoseData pose_ss_T_device;
    TangoCoordinateFramePair frame_pair;
    frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoService_getPoseAtTime(timestamp, frame_pair, &pose_ss_T_device);

    if(pose_ss_T_device.status_code != TANGO_POSE_VALID) {
      LOGE("ss_T_device pose invalid");
    }

    return GetMatrixFromPose(pose_ss_T_device);
  }

  void PoseData::SetColorCameraIntrinsics(TangoCameraIntrinsics color_camera_intrinsics) {
    color_camera_intrinsics_ = color_camera_intrinsics;
  }

  TangoCameraIntrinsics PoseData::GetColorCameraIntrinsics() {
    return color_camera_intrinsics_;
  }

  glm::mat4 PoseData::GetImuTDevice() {
    return imu_T_device_;
  }

  glm::mat4 PoseData::GetImuTDepthCamera() {
    return imu_T_depth_camera_;
  }

  glm::mat4 PoseData::GetSSTColorCamera(double timestamp) {
    glm::mat4 ss_T_device = GetPoseAtTime(timestamp);
    return ss_T_device * device_T_color_camera_;
  }

  glm::mat4 PoseData::GetSSTDepthCamera(double timestamp) {
    glm::mat4 ss_T_device = GetPoseAtTime(timestamp);
    return ss_T_device * device_T_depth_camera_;
  }

  glm::mat4 PoseData::GetColorCameraTDepthCamera(double color_timestamp, double depth_timestamp) {
    glm::mat4 ss_T_device_color_timestamp = GetPoseAtTime(color_timestamp);
    glm::mat4 ss_T_device_depth_timestamp = GetPoseAtTime(depth_timestamp);

    return color_camera_T_device_ * glm::inverse(ss_T_device_color_timestamp) * ss_T_device_depth_timestamp * device_T_depth_camera_;
  }

  glm::mat4 PoseData::GetExtrinsicsAppliedOpenGLWorldFrame(
      const glm::mat4 pose_matrix) {
    // This full multiplication is equal to:
    //   opengl_world_T_opengl_camera =
    //      opengl_world_T_start_service *
    //      start_service_T_device *
    //      device_T_imu *
    //      imu_T_depth_camera *
    //      depth_camera_T_opengl_camera;
    //
    // More information about frame transformation can be found here:
    // Frame of reference:
    //   https://developers.google.com/project-tango/overview/frames-of-reference
    // Coordinate System Conventions:
    //   https://developers.google.com/project-tango/overview/coordinate-systems
    return tango_gl::conversions::opengl_world_T_tango_world() * pose_matrix;
  }

  glm::mat4 PoseData::GetMatrixFromPose(const TangoPoseData& pose) {
    // Convert pose data to vec3 for position and quaternion for orientation.
    //
    // More information about frame transformation can be found here:
    // Frame of reference:
    //   https://developers.google.com/project-tango/overview/frames-of-reference
    // Coordinate System Conventions:
    //   https://developers.google.com/project-tango/overview/coordinate-systems
    glm::vec3 translation =
        glm::vec3(pose.translation[0], pose.translation[1], pose.translation[2]);
    glm::quat rotation = glm::quat(pose.orientation[3], pose.orientation[0],
                                   pose.orientation[1], pose.orientation[2]);
    glm::mat4 matrix =
        glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(rotation);
    return matrix;
  }

  std::string PoseData::GetStringFromStatusCode(TangoPoseStatusType status) {
    std::string ret_string;
    switch (status) {
      case TANGO_POSE_INITIALIZING:
        ret_string = "initializing";
        break;
      case TANGO_POSE_VALID:
        ret_string = "valid";
        break;
      case TANGO_POSE_INVALID:
        ret_string = "invalid";
        break;
      case TANGO_POSE_UNKNOWN:
        ret_string = "unknown";
        break;
      default:
        ret_string = "status_code_invalid";
        break;
    }
    return ret_string;
  }

  void PoseData::FormatPoseString() {
    std::stringstream string_stream;
    string_stream.setf(std::ios_base::fixed, std::ios_base::floatfield);
    string_stream.precision(3);
    string_stream << "status: " << GetStringFromStatusCode(cur_pose_.status_code)
    << ", count: " << pose_counter_ << ", delta time (ms): "
    << (cur_pose_.timestamp - prev_pose_.timestamp) *
       kMeterToMillimeter << ", position (m): ["
    << cur_pose_.translation[0] << ", " << cur_pose_.translation[1]
    << ", " << cur_pose_.translation[2] << "]"
    << ", orientation: [" << cur_pose_.orientation[0] << ", "
    << cur_pose_.orientation[1] << ", " << cur_pose_.orientation[2]
    << ", " << cur_pose_.orientation[3] << "]";
    pose_string_ = string_stream.str();
    string_stream.flush();
  }

}  // namespace tango_point_cloud
