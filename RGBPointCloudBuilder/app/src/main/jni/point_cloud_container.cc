#include "rgb-depth-sync/point_cloud_container.h"
#include <tango-gl/conversions.h>
#include <Eigen/Dense>

namespace rgb_depth_sync {

  PointCloudContainer::PointCloudContainer() {
    Set();
  }

  PointCloudContainer::~PointCloudContainer(){}

  void PointCloudContainer::UpdateMergedPointCloud() {

    /*LOGE("update merged pcd");
    TangoPoseData pose_start_service_T_device_t1;

    TangoCoordinateFramePair color_frame_pair;
    color_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    color_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;

    for (size_t j = 0; j < pcd_container_.size(); j++) {
      LOGE("get pose for pcd");
      if (TangoService_getPoseAtTime(pcd_container_[j].first, color_frame_pair,
                                     &pose_start_service_T_device_t1) != TANGO_SUCCESS) {
        LOGE("SynchronizationApplication: Could not find a valid pose at time %lf"
                " for the color camera.", pcd_container_[j].first);
      }

      glm::mat4 start_service_T_device_t1 = util::GetMatrixFromPose(&pose_start_service_T_device_t1);
      glm::mat4 start_service_T_color_t1 = start_service_T_device_t1 * device_T_color_;

      glm::vec3 translation = util::GetTranslationFromMatrix(start_service_T_color_t1);
      glm::quat rotation = util::GetRotationFromMatrix(start_service_T_color_t1);

      glm::vec3 xyz;
      glm::vec3 xyz_transformed;

      LOGE("transform pcd with pose and save...");
      LOGE("size of pcd_container: %i", pcd_container_.size());
      LOGE("size of pcd: %i", pcd_container_[j].second.size());
      for (int i = 0; i < pcd_container_[j].second.size()-4; i=i+4) {
        xyz.x = pcd_container_[j].second[i];
        xyz.y = pcd_container_[j].second[i+1];
        xyz.z = pcd_container_[j].second[i+2];

        xyz_transformed = rotation * xyz;
        xyz_transformed.x += translation.x;
        xyz_transformed.y += translation.y;
        xyz_transformed.z += translation.z;

        merged_pcd_.push_back(xyz_transformed.x);
        merged_pcd_.push_back(xyz_transformed.y);
        merged_pcd_.push_back(xyz_transformed.z);
        merged_pcd_.push_back(pcd_container_[j].second[i+3]); // stuffed rgb value
      }
      LOGE("save stop...");
    }*/
  }

  void PointCloudContainer::Set() {
    TangoPoseData pose_imu_T_device;
    TangoPoseData pose_imu_T_color;
    TangoPoseData pose_imu_T_depth;
    TangoCoordinateFramePair frame_pair;

    // Extrinsic transformation between the device and the imu coordinate frame.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    int ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_device);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get transform between the IMU "
              "and"
              "device frames. Something is wrong with device extrinsics.");
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
    }

    // Extrinsic transformation between the depth camera and the imu coordinate frame.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_depth);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get transform between the IMU "
              "and depth camera frames. Something is wrong with device extrinsics.");
    }

    glm::mat4 imu_T_device = util::GetMatrixFromPose(&pose_imu_T_device);
    glm::mat4 imu_T_color = util::GetMatrixFromPose(&pose_imu_T_color);
    glm::mat4 imu_T_depth = util::GetMatrixFromPose(&pose_imu_T_depth);

    device_T_color_ = glm::inverse(imu_T_device) * imu_T_color;
    device_T_depth_ = glm::inverse(imu_T_device) * imu_T_depth;
    color_T_device_ = glm::inverse(device_T_color_);
  }

  void PointCloudContainer::SavePointCloud(TangoPoseData* ss_T_device_dts,
                                           TangoPoseData* ss_T_device_cts,
                                           const std::vector <float> &render_point_cloud_buffer,
                                           const std::vector <uint8_t> &rgb_map_buffer) {
    //LOGE("save pcd");
    previous_point_cloud_ = current_point_cloud_;
    current_point_cloud_ = render_point_cloud_buffer;

    glm::mat4 start_service_T_device_dts = util::GetMatrixFromPose(&(*ss_T_device_dts));
    glm::mat4 start_service_T_device_cts = util::GetMatrixFromPose(&(*ss_T_device_cts));
    glm::mat4 start_service_T_color_cts = start_service_T_device_cts * device_T_color_;

    glm::mat4 color_T_depth_ =
        glm::inverse(device_T_color_) *
        glm::inverse(start_service_T_device_cts) *
        start_service_T_device_dts *
        device_T_depth_;

    glm::mat4 opengl_T_depth_ =
        tango_gl::conversions::opengl_world_T_tango_world() *
        start_service_T_color_cts *
        glm::inverse(device_T_color_) *
        glm::inverse(start_service_T_device_cts) *
        start_service_T_device_dts *
        device_T_depth_;

    for (int i = 0; i < render_point_cloud_buffer.size() - 3; i = i + 3) {
      float x = render_point_cloud_buffer[i];
      float y = render_point_cloud_buffer[i + 1];
      float z = render_point_cloud_buffer[i + 2];

      // transform depth point to color frame
      glm::vec3 color_point = glm::vec3(color_T_depth_ * glm::vec4(x, y, z, 1.0));
      glm::vec3 opengl_point = glm::vec3(opengl_T_depth_ * glm::vec4(x, y, z, 1.0));

      int pixel_x, pixel_y;
      // get the coordinate on image plane.
      pixel_x = static_cast<int>((rgb_camera_intrinsics_.fx) *
                                 (color_point.x / color_point.z) +
                                 rgb_camera_intrinsics_.cx);

      pixel_y = static_cast<int>((rgb_camera_intrinsics_.fy) *
                                 (color_point.y / color_point.z) +
                                 rgb_camera_intrinsics_.cy);

      if (!(pixel_x >= 0 && pixel_x <= rgb_camera_intrinsics_.width && pixel_y >= 0 && pixel_y <= rgb_camera_intrinsics_.height)) {
        //LOGE("pixel_x: %i", pixel_x);
        //LOGE("pixel_y: %i", pixel_y);
        continue;
      }

      std::vector <uint8_t> rgb;
      size_t index = (pixel_x + pixel_y * rgb_camera_intrinsics_.width) * 3;

      rgb.push_back(rgb_map_buffer[index]);
      rgb.push_back(rgb_map_buffer[index + 1]);
      rgb.push_back(rgb_map_buffer[index + 2]);

      Point point(opengl_point, ss_T_device_dts, rgb);
      pcd_container_.push_back(point);
    }
  }

  const std::vector <float> PointCloudContainer::GetMergedPointCloud() const {
    return merged_pcd_;
  }

  std::vector<float> PointCloudContainer::GetVertices() {
    vertices_.clear();

    for(int i = 0; i < pcd_container_.size(); i++) {
      vertices_.push_back(pcd_container_[i].point_.x);
      vertices_.push_back(pcd_container_[i].point_.y);
      vertices_.push_back(pcd_container_[i].point_.z);
    }

    return vertices_;
  }

  std::vector<uint8_t> PointCloudContainer::GetRGBValues() {
    rgb_values_.clear();

    for(int i = 0; i < pcd_container_.size(); i++) {
      rgb_values_.push_back(pcd_container_[i].rgb_[0]);
      rgb_values_.push_back(pcd_container_[i].rgb_[1]);
      rgb_values_.push_back(pcd_container_[i].rgb_[2]);
    }

    return rgb_values_;
  }

  /*float PointCloudContainer::Hash(float x, float y, float z) {
    float p1 = 73856093;
    float p2 = 19349663;
    float p3 = 83492791;
    float n = 2001;
    float key = ((x * p1) xor (y * p2) xor (z * p3)) % n;
  }*/

  const std::vector<float> PointCloudContainer::GetPreviousPointCloud() const {
    return previous_point_cloud_;
  }

  const std::vector<float> PointCloudContainer::GetCurrentPointCloud() const {
    return current_point_cloud_;
  }

  void PointCloudContainer::SetCameraIntrinsics(TangoCameraIntrinsics intrinsics) {
    rgb_camera_intrinsics_ = intrinsics;
  }

} // namespace rgb_depth_sync