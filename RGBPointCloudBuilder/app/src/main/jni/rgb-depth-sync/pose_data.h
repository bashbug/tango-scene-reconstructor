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

#ifndef TANGO_POINT_CLOUD_POSE_DATA_H_
#define TANGO_POINT_CLOUD_POSE_DATA_H_

#include <jni.h>
#include <mutex>
#include <sstream>
#include <tango_client_api.h>  // NOLINT
#include <tango-gl/conversions.h>
#include <tango-gl/util.h>

namespace rgb_depth_sync {

  class PoseData {
    public:
      static PoseData* GetInstance();
      void UpdatePose(const TangoPoseData* pose_data);
      void SetImuTDevice(const glm::mat4& imu_T_device);
      void SetImuTDepthCamera(const glm::mat4& imu_T_depth_camera);
      void SetImuTColorCamera(const glm::mat4& imu_T_color);
      void SetDeviceTColorCamera(const glm::mat4& device_T_color);
      void SetDeviceTDepthCamera(const glm::mat4& device_T_depth);
      void SetColorCameraTDevice(const glm::mat4& color_T_device);
      void SetColorCameraIntrinsics(TangoCameraIntrinsics color_camera_intrinsics);
      void SetDepthCameraIntrinsics(TangoCameraIntrinsics depth_camera_intrinsics);
      TangoCameraIntrinsics GetColorCameraIntrinsics();
      TangoCameraIntrinsics GetDepthCameraIntrinsics();
      std::string GetPoseDebugString();
      glm::mat4 GetLatestPoseMatrix();
      glm::mat4 GetPoseAtTime(double timestamp);
      glm::mat4 GetImuTDevice();
      glm::mat4 GetImuTDepthCamera();
      glm::mat4 GetSSTColorCamera(double timestamp);
      glm::mat4 GetSSTDepthCamera(double timestamp);
      glm::mat4 GetColorCameraTDepthCamera(double color_timestamp, double depth_timestamp);
      glm::mat4 GetMatrixFromPose(const TangoPoseData& pose);
      glm::mat4 GetExtrinsicsAppliedOpenGLWorldFrame(const glm::mat4 pose_matrix);

    private:
      PoseData() {};
      PoseData(PoseData const&) {};
      void operator = (PoseData const&) {};
      std::string GetStringFromStatusCode(TangoPoseStatusType status);
      void FormatPoseString();
      glm::mat4 imu_T_device_;
      glm::mat4 imu_T_depth_camera_;
      glm::mat4 device_T_color_camera_;
      glm::mat4 device_T_depth_camera_;
      glm::mat4 color_camera_T_device_;
      glm::mat4 imu_T_color_camera_;
      TangoPoseData cur_pose_;
      TangoPoseData prev_pose_;
      TangoCameraIntrinsics color_camera_intrinsics_;
      TangoCameraIntrinsics depth_camera_intrinsics_;
      std::string pose_string_;
      size_t pose_counter_;
  };
}  // namespace tango_point_cloud

#endif  // TANGO_POINT_CLOUD_POSE_DATA_H_
