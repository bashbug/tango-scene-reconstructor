#include "rgb-depth-sync/pcd.h"

namespace rgb_depth_sync {

  PCD::PCD() {
  }

  PCD::~PCD() {
    LOGE("PointCloudData is destroyed...");
  }

  void PCD::SetTranslation(const glm::vec3& translation) {
    translation_ = translation;
  }

  void PCD::SetRotation(const glm::quat rotation) {
    rotation_ = rotation;
  }

  void PCD::SetKeyPointsAndDescriptors(const std::vector<cv::KeyPoint>& frame_key_points, cv::Mat frame_descriptors) {
    frame_key_points_ = frame_key_points;
    frame_descriptors_ = frame_descriptors;
  }

  void PCD::SetFrame(const cv::Mat& frame) {
    frame_ = frame;
  }

  void PCD::MapXYZWithRGB(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >& xyz,
                          const std::vector<uint8_t>& rgb,
                          double xyz_timestamp,
                          double rgb_timestamp) {

    xyz_timestamp_ = xyz_timestamp;
    rgb_timestamp_ = rgb_timestamp;

    PoseData* pose_data_ = PoseData::GetInstance();
    pose_ = pose_data_->GetSSTColorCamera(xyz_timestamp);  // ss_T_color
    TangoCameraIntrinsics color_camera_intrinsics = pose_data_->GetColorCameraIntrinsics();

    // transform the depth points into the color frame with relative pose
    glm::mat4 color_T_depth = pose_data_->GetColorCameraTDepthCamera(rgb_timestamp, xyz_timestamp);

    translation_ = util::GetTranslationFromMatrix(pose_);
    rotation_ = util::GetRotationFromMatrix(pose_);

    size_t xyz_size = xyz.size();
    size_t rgb_size = rgb.size();

    for (int i = 0; i < xyz_size; i++) {
      int pixel_x, pixel_y;

      // transform depth point to color frame
      glm::vec3 color_point = glm::vec3(color_T_depth * glm::vec4(xyz[i].x, xyz[i].y, xyz[i].z, 1.0f));

      glm::vec3 ss_point = glm::vec3(pose_ * glm::vec4 (color_point.x, color_point.y, color_point.z, 1.0f));

      pixel_x = static_cast<int>(color_point.x / color_point.z * color_camera_intrinsics.fx +
                                 color_camera_intrinsics.cx);

      pixel_y = static_cast<int>(color_point.y / color_point.z * color_camera_intrinsics.fy +
                                 color_camera_intrinsics.cy);

      /*pixel_x = static_cast<int>(xyz[i] / xyz[i+2] * color_camera_intrinsics.fx +
                                 color_camera_intrinsics.cx);

      pixel_y = static_cast<int>(xyz[i+1] / xyz[i+2] * color_camera_intrinsics.fy +
                                 color_camera_intrinsics.cy);*/

      if (pixel_x < 0 || pixel_x > color_camera_intrinsics.width || pixel_y < 0 || pixel_y > color_camera_intrinsics.height)
        continue;

      size_t index = pixel_x + pixel_y * color_camera_intrinsics.width;

      if (index * 3 + 2 >= rgb_size)
        continue;

      xyz_values_color_camera_.push_back(color_point.x);
      xyz_values_color_camera_.push_back(color_point.y);
      xyz_values_color_camera_.push_back(color_point.z);

      pcd_.push_back(color_point.x);
      pcd_.push_back(color_point.y);
      pcd_.push_back(color_point.z);

      xyz_values_ss_.push_back(ss_point.x);
      xyz_values_ss_.push_back(ss_point.y);
      xyz_values_ss_.push_back(ss_point.z);

      rgb_values_.push_back(rgb[index * 3]);
      rgb_values_.push_back(rgb[index * 3 + 1]);
      rgb_values_.push_back(rgb[index * 3 + 2]);


      uint32_t tmp = ((uint32_t) (rgb[index * 3])) << 16 | ((uint32_t) (rgb[index * 3 + 1])) << 8 | ((uint32_t) (rgb[index * 3 +2]));

      pcd_.push_back(*reinterpret_cast<float *>(&tmp));
    }
  }

  std::vector<float> PCD::GetXYZValues() {
    return xyz_values_color_camera_;
  }

  std::vector<uint8_t> PCD::GetRGBValues() {
    return rgb_values_;
  }

  glm::mat4 PCD::GetPose() {
    return pose_;
  }

  glm::vec3 PCD::GetTranslation() {
    return translation_;
  }

  glm::quat PCD::GetRotation() {
    return rotation_;
  }

  std::vector<float> PCD::GetXYZValuesTSS() {
    return xyz_values_ss_;
  }

  std::vector<float> PCD::GetPCD() {
    return pcd_;
  }

  std::vector<cv::KeyPoint> PCD::GetFrameKeyPoints() {
    return frame_key_points_;
  }

  cv::Mat PCD::GetFrameDescriptors() {
    return frame_descriptors_;
  }

  cv::Mat PCD::GetFrame() {
    return frame_;
  }
} // namespace rgb_depth_sync