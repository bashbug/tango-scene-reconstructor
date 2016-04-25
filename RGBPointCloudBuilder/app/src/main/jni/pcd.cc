#include "rgb-depth-sync/pcd.h"

namespace rgb_depth_sync {

  PCD::PCD() {
    cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_transformed_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pose_data_ = PoseData::GetInstance();
  }

  PCD::~PCD() {
    LOGE("PointCloudData is destroyed...");
  }

  void PCD::SetTranslation(const glm::vec3& translation) {
    translation_ = translation;
  }

  void PCD::SetRotation(const glm::quat& rotation) {
    rotation_ = rotation;
  }

  void PCD::SetTranslationSM(const glm::vec3& translation) {
    translation_sm_ = translation;
  }

  void PCD::SetRotationMSM(const glm::quat& rotation) {
    rotation_msm_ = rotation;
  }

  void PCD::SetTranslationMSM(const glm::vec3& translation) {
    translation_msm_ = translation;
  }

  void PCD::SetRotationSM(const glm::quat& rotation) {
    rotation_sm_ = rotation;
  }

  void PCD::SetKeyPointsAndDescriptors(const std::vector<cv::KeyPoint>& frame_key_points, cv::Mat frame_descriptors) {
    frame_key_points_ = frame_key_points;
    frame_descriptors_ = frame_descriptors;
  }

  void PCD::SetFrame(const cv::Mat& frame) {
    frame_ = frame;
  }

  void PCD::SetRGBImage(const cv::Mat& rgb_image) {
    rgb_image_ = rgb_image;
  }

  void PCD::SetSMPose(Eigen::Isometry3f sm_pose) {
    sm_pose_ = sm_pose;
  }

  void PCD::SetMSMPose(Eigen::Isometry3f msm_pose) {
    msm_pose_ = msm_pose;
  }

  void PCD::MapXYZWithRGB(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >& xyz,
                          const std::vector<uint8_t>& rgb,
                          double xyz_timestamp,
                          double rgb_timestamp) {

    xyz_timestamp_ = xyz_timestamp;
    rgb_timestamp_ = rgb_timestamp;

    pose_ = pose_data_->GetSSTColorCamera(xyz_timestamp);  // ss_T_color
    TangoCameraIntrinsics color_camera_intrinsics = pose_data_->GetColorCameraIntrinsics();
    TangoCameraIntrinsics depth_camera_intrinsics = pose_data_->GetDepthCameraIntrinsics();

    // transform the depth points into the color frame with relative pose
    glm::mat4 color_T_depth = pose_data_->GetColorCameraTDepthCamera(rgb_timestamp, xyz_timestamp);

    translation_ = util::GetTranslationFromMatrix(pose_);
    rotation_ = util::GetRotationFromMatrix(pose_);

    cloud_->sensor_origin_[0] = translation_[0];
    cloud_->sensor_origin_[1] = translation_[1];
    cloud_->sensor_origin_[2] = translation_[2];

    // Eigen::Quaternionf(w, x, y, z);
    cloud_->sensor_orientation_ = Eigen::Quaternionf(rotation_.w, rotation_.x, rotation_.y, rotation_.z);

    float k1 = static_cast<float>(depth_camera_intrinsics.distortion[0]);
    float k2 = static_cast<float>(depth_camera_intrinsics.distortion[1]);
    float k3 = static_cast<float>(depth_camera_intrinsics.distortion[2]);

    float c_k1 = static_cast<float>(color_camera_intrinsics.distortion[0]);
    float c_k2 = static_cast<float>(color_camera_intrinsics.distortion[1]);
    float c_k3 = static_cast<float>(color_camera_intrinsics.distortion[2]);

    int xyz_size = xyz.size();
    int rgb_size = rgb.size();

    for (int i = 0; i < xyz_size; i++) {
      int pixel_x, pixel_y;
      pcl::PointXYZRGB p;
      pcl::PointXYZ depth_p;

      // remove distortion from xyz[i] values

      float ru = sqrt((pow(xyz[i].x, 2) + pow(xyz[i].y, 2)) / pow(xyz[i].z, 2));
      float rd = ru + k1 * pow(ru, 3) + k2 * pow(ru, 5) + k3 * pow(ru, 7);

      int x = static_cast<int>(xyz[i].x / xyz[i].z * depth_camera_intrinsics.fx * rd / ru + depth_camera_intrinsics.cx);
      int y = static_cast<int>(xyz[i].y / xyz[i].z * depth_camera_intrinsics.fy * rd / ru + depth_camera_intrinsics.cy);

      depth_p.x = (xyz[i].z*(x-depth_camera_intrinsics.cx)) / depth_camera_intrinsics.fx;
      depth_p.y = (xyz[i].z*(y-depth_camera_intrinsics.cy)) / depth_camera_intrinsics.fy;
      depth_p.z = xyz[i].z;

      // transform depth point to color frame
      glm::vec3 color_point = glm::vec3(
          color_T_depth * glm::vec4(xyz[i].x, xyz[i].y, xyz[i].z, 1.0f));

      pixel_x = static_cast<int>(color_point.x / color_point.z * color_camera_intrinsics.fx + color_camera_intrinsics.cx);
      pixel_y = static_cast<int>(color_point.y / color_point.z * color_camera_intrinsics.fy + color_camera_intrinsics.cy);

      glm::vec3 ss_point = glm::vec3(pose_ * glm::vec4(color_point.x, color_point.y, color_point.z, 1.0f));

      if (pixel_x < 0 || pixel_x > color_camera_intrinsics.width ||
          pixel_y < 0 || pixel_y > color_camera_intrinsics.height)
        continue;

      int index = pixel_x + pixel_y * color_camera_intrinsics.width;

      if (index * 3 + 2 >= rgb_size)
        continue;

      p.x = color_point.x;
      p.y = color_point.y;
      p.z = color_point.z;
      p.r = rgb[index * 3];
      p.g = rgb[index * 3 + 1];
      p.b = rgb[index * 3 + 2];
      cloud_->points.push_back(p);

      p.x = ss_point.x;
      p.y = ss_point.y;
      p.z = ss_point.z;
      p.r = rgb[index * 3];
      p.g = rgb[index * 3 + 1];
      p.b = rgb[index * 3 + 2];
      cloud_transformed_->points.push_back(p);

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

      uint32_t tmp =
          ((uint32_t)(rgb[index * 3])) << 16 | ((uint32_t)(rgb[index * 3 + 1])) << 8 |
          ((uint32_t)(rgb[index * 3 + 2]));

      pcd_.push_back(*reinterpret_cast<float *>(&tmp));
    }

    cloud_->height = 1;
    cloud_->width = cloud_->points.size();
    cloud_transformed_->height = 1;
    cloud_transformed_->width = cloud_->points.size();
    cloud_transformed_->sensor_origin_.setZero();
    cloud_transformed_->sensor_orientation_ = Eigen::Quaternionf::Identity();
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

  Eigen::Isometry3f PCD::GetSMPose() {
    return sm_pose_;
  }

  Eigen::Isometry3f PCD::GetMSMPose() {
    return msm_pose_;
  }

  glm::vec3 PCD::GetTranslation() {
    return translation_;
  }

  glm::quat PCD::GetRotation() {
    return rotation_;
  }

  glm::vec3 PCD::GetTranslationSM() {
    return translation_sm_;
  }

  glm::quat PCD::GetRotationSM() {
    return rotation_sm_;
  }

  glm::vec3 PCD::GetTranslationMSM() {
    return translation_msm_;
  }

  glm::quat PCD::GetRotationMSM() {
    return rotation_msm_;
  }

  std::vector<float> PCD::GetXYZValuesTSS() {
    return xyz_values_ss_;
  }

  std::vector<float> PCD::GetPCD() {
    return pcd_;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCD::GetPointCloudTransformed() {
    return cloud_transformed_;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCD::GetPointCloud() {
    return cloud_;
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

  void PCD::SaveRGBImage(const char* path, int id) {
    char filename[1024];
    sprintf(filename, "%s/%05d.jpg", path, id);
    cv::Mat tmp;
    cv::cvtColor(rgb_image_, tmp, CV_RGB2BGR);
    cv::imwrite(filename, tmp);
  }
} // namespace rgb_depth_sync