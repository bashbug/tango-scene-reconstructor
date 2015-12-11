//
// Created by anastasia on 09.12.15.
//

#include "rgb-depth-sync/point.h"

namespace rgb_depth_sync {

  Point::Point(glm::vec3 point, TangoPoseData* pose_pointer, std::vector<uint8_t> &rgb) {
    pose_pointer_ = pose_pointer;
    UpdatePose();
    point_ = point;
    //TransformPoint();
    rgb_ = rgb;
  }

  Point::~Point() {
  }

  void Point::UpdatePoint() {
    point_ = glm::inverse(current_rotation_) * point_;
    point_.x -= current_translation_.x;
    point_.y -= current_translation_.y;
    point_.z -= current_translation_.z;
    TransformPoint();
    UpdatePose();
  }

  void Point::TransformPoint() {
    point_ = current_rotation_ * point_;
    point_.x += current_translation_.x;
    point_.y += current_translation_.y;
    point_.z += current_translation_.z;
  }

  void Point::UpdatePose() {
    glm::mat4 pose_mat = util::GetMatrixFromPose(&(*pose_pointer_));
    current_translation_ = util::GetTranslationFromMatrix(pose_mat);
    current_rotation_ = util::GetRotationFromMatrix(pose_mat);
  }

  void Point::Set() {
    /*TangoPoseData pose_imu_T_device;
    TangoPoseData pose_imu_T_color;
    TangoPoseData pose_imu_T_depth;
    TangoCoordinateFramePair frame_pair;

    // Extrinsic transformation between the device and the imu coordinate frame.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_device);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get transform between the IMU "
              "and"
              "device frames. Something is wrong with device extrinsics.");
      return ret;
    }

    // Extrinsic transformation between the color camera and the imu coordinate frame.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_color);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get transform between the IMU "
              "and"
              "color camera frames. Something is wrong with device extrinsics.");
      return ret;
    }

    // Extrinsic transformation between the depth camera and the imu coordinate frame.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_depth);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get transform between the IMU "
              "and depth camera frames. Something is wrong with device extrinsics.");
      return ret;
    }

    glm::mat4 imu_T_device = util::GetMatrixFromPose(&pose_imu_T_device);
    glm::mat4 imu_T_color = util::GetMatrixFromPose(&pose_imu_T_color);
    glm::mat4 imu_T_depth = util::GetMatrixFromPose(&pose_imu_T_depth);

    device_T_color_ = glm::inverse(imu_T_device) * imu_T_color;
    device_T_depth_ = glm::inverse(imu_T_device) * imu_T_depth;
    color_T_device_ = glm::inverse(device_T_color_);*/
  }
}