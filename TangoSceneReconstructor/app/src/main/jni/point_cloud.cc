#include "tango-scene-reconstructor/point_cloud.h"

namespace tango_scene_reconstructor {

  PointCloud::PointCloud(float far_clipping) {
    cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_transformed_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pose_data_ = PoseData::GetInstance();

    yuv_frame_.create(720*3/2, 1280, CV_8UC1);
    rgb_frame_.create(720, 1280, CV_8UC3);
    yuv_size_ = 720*3/2*1280;
    rgb_size_ = 720*1280*3;

    // default frustum clipping
    near_clipping_ = 0.25;
    far_clipping_ = far_clipping;
  }

  PointCloud::~PointCloud() { }

  void PointCloud::SetXYZ(TangoXYZij* XYZij) {
    TangoSupport_createXYZij(XYZij->xyz_count, &XYZij_);
    TangoSupport_copyXYZij(XYZij, &XYZij_);
  }

  void PointCloud::SetYUV(TangoImageBuffer* YUV) {
    YUV_.data = YUV->data;
    YUV_.width = YUV->width;
    YUV_.height = YUV->height;
    YUV_.stride = YUV->stride;
    YUV_.format = YUV->format;
    YUV_.frame_number = YUV->frame_number;
    YUV_.timestamp = YUV->timestamp;
    SetRGB();
  }

  void PointCloud::SetRGB() {
    memcpy(yuv_frame_.data, YUV_.data, yuv_size_);
    cv::cvtColor(yuv_frame_, rgb_frame_, CV_YUV2RGB_NV21);
  }

  TangoXYZij* PointCloud::GetXYZ() {
    return &XYZij_;
  }

  TangoImageBuffer* PointCloud::GetYUV() {
    return &YUV_;
  }

  void PointCloud::Update() {

    pose_ = pose_data_->GetSSTColorCamera(XYZij_.timestamp);  // TODO: could be a bug!
    yuv_pose_ = pose_data_->GetSSTColorCamera(YUV_.timestamp);
    xyz_pose_ = pose_data_->GetSSTDepthCamera(XYZij_.timestamp);
    TangoCameraIntrinsics color_camera_intrinsics = pose_data_->GetColorCameraIntrinsics();
    TangoCameraIntrinsics depth_camera_intrinsics = pose_data_->GetDepthCameraIntrinsics();

    // transform the depth points into the color frame with relative pose
    glm::mat4 color_T_depth = pose_data_->GetColorCameraTDepthCamera(YUV_.timestamp, XYZij_.timestamp);

    translation_ = util::GetTranslationFromMatrix(pose_);
    rotation_ = util::GetRotationFromMatrix(pose_);

    cloud_->sensor_origin_[0] = translation_[0];
    cloud_->sensor_origin_[1] = translation_[1];
    cloud_->sensor_origin_[2] = translation_[2];
    cloud_->sensor_orientation_ = Eigen::Quaternionf(rotation_.w, rotation_.x, rotation_.y, rotation_.z);

    for (int i = 0; i < XYZij_.xyz_count; i++) {

      if (XYZij_.xyz[i][2] < near_clipping_ || XYZij_.xyz[i][2] > far_clipping_)
        continue;

      int pixel_x, pixel_y;
      pcl::PointXYZRGB p;

      // transform depth point to color frame
      glm::vec3 color_point = glm::vec3(color_T_depth * glm::vec4(XYZij_.xyz[i][0], XYZij_.xyz[i][1], XYZij_.xyz[i][2], 1.0f));

      pixel_x = static_cast<int>(color_point.x / color_point.z * color_camera_intrinsics.fx + color_camera_intrinsics.cx);
      pixel_y = static_cast<int>(color_point.y / color_point.z * color_camera_intrinsics.fy + color_camera_intrinsics.cy);

      glm::vec3 ss_point = glm::vec3(pose_ * glm::vec4(color_point.x, color_point.y, color_point.z, 1.0f));

      if (pixel_x < 0 || pixel_x > color_camera_intrinsics.width ||
          pixel_y < 0 || pixel_y > color_camera_intrinsics.height)
        continue;

      int index = pixel_x + pixel_y * color_camera_intrinsics.width;

      if (index * 3 + 2 >= rgb_size_)
        continue;

      p.x = color_point.x;
      p.y = color_point.y;
      p.z = color_point.z;
      p.r = rgb_frame_.data[index * 3];
      p.g = rgb_frame_.data[index * 3 + 1];
      p.b = rgb_frame_.data[index * 3 + 2];
      cloud_->points.push_back(p);

      p.x = ss_point.x;
      p.y = ss_point.y;
      p.z = ss_point.z;
      cloud_transformed_->points.push_back(p);

      xyz_values_color_camera_.push_back(color_point.x);
      xyz_values_color_camera_.push_back(color_point.y);
      xyz_values_color_camera_.push_back(color_point.z);

      xyz_values_ss_.push_back(ss_point.x);
      xyz_values_ss_.push_back(ss_point.y);
      xyz_values_ss_.push_back(ss_point.z);

      rgb_values_.push_back(rgb_frame_.data[index * 3]);
      rgb_values_.push_back(rgb_frame_.data[index * 3 + 1]);
      rgb_values_.push_back(rgb_frame_.data[index * 3 + 2]);

    }

    cloud_->height = 1;
    cloud_->width = cloud_->points.size();
    cloud_transformed_->height = 1;
    cloud_transformed_->width = cloud_->points.size();
    cloud_transformed_->sensor_origin_.setZero();
    cloud_transformed_->sensor_orientation_ = Eigen::Quaternionf::Identity();
  }

  void PointCloud::SetTranslation(const glm::vec3& translation) {
    translation_ = translation;
  }

  void PointCloud::SetRotation(const glm::quat& rotation) {
    rotation_ = rotation;
  }

  void PointCloud::SetTranslationFTFSM(const glm::vec3& translation) {
    translation_ftfsm_ = translation;
  }

  void PointCloud::SetRotationMFSM(const glm::quat& rotation) {
    rotation_mfsm_ = rotation;
  }

  void PointCloud::SetTranslationMFSM(const glm::vec3& translation) {
    translation_mfsm_ = translation;
  }

  void PointCloud::SetRotationFTFSM(const glm::quat& rotation) {
    rotation_ftfsm_ = rotation;
  }

  void PointCloud::SaveAsPCD(const char* filename) {
    pcl::PointCloud<pcl::PointXYZRGB> out;
    out = *cloud_;
    out.sensor_origin_ = Eigen::Vector4f (translation_.x, translation_.y, translation_.z, 1.0f);
    out.sensor_orientation_ = Eigen::Quaternionf(rotation_.w, rotation_.x, rotation_.y, rotation_.z);
    pcl::io::savePCDFile(filename, out);
  }

  void PointCloud::SaveAsPCDWithMFSMPose(const char* filename) {
    pcl::PointCloud<pcl::PointXYZRGB> out;
    out = *cloud_;
    out.sensor_origin_ = Eigen::Vector4f (translation_mfsm_.x, translation_mfsm_.y, translation_mfsm_.z, 1.0f);
    out.sensor_orientation_ = Eigen::Quaternionf(rotation_mfsm_.w, rotation_mfsm_.x, rotation_mfsm_.y, rotation_mfsm_.z);
    pcl::io::savePCDFile(filename, out);
  }

  void PointCloud::SaveAsPCDWithFTFSMPose(const char* filename) {
    pcl::PointCloud<pcl::PointXYZRGB> out;
    out = *cloud_;
    out.sensor_origin_ = Eigen::Vector4f (translation_ftfsm_.x, translation_ftfsm_.y, translation_ftfsm_.z, 1.0f);
    out.sensor_orientation_ = Eigen::Quaternionf(rotation_ftfsm_.w, rotation_ftfsm_.x, rotation_ftfsm_.y, rotation_ftfsm_.z);
    pcl::io::savePCDFile(filename, out);  }

  void PointCloud::SetFTFSMPose(Eigen::Isometry3f ftfsm_pose) {
    ftfsm_pose_ = ftfsm_pose;
  }

  void PointCloud::SetMFSMPose(Eigen::Isometry3f mfsm_pose) {
    mfsm_pose_ = mfsm_pose;
  }

  void PointCloud::SetNearClipping(float near_clipping) {
    near_clipping_ = near_clipping;
  }

  void PointCloud::SetFarClipping(float far_clipping) {
    far_clipping_ = far_clipping;
  }

  void PointCloud::RemoveOutliers(float radius) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_);
    sor.setMeanK(radius);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    cloud_ = cloud_filtered;
  }

  std::vector<float> PointCloud::GetXYZValues() {
    return xyz_values_color_camera_;
  }

  std::vector<uint8_t> PointCloud::GetRGBValues() {
    return rgb_values_;
  }

  glm::mat4 PointCloud::GetPose() {
    return pose_;
  }

  Eigen::Isometry3f PointCloud::GetFTFSMPose() {
    return ftfsm_pose_;
  }

  Eigen::Isometry3f PointCloud::GetMFSMPose() {
    return mfsm_pose_;
  }

  glm::vec3 PointCloud::GetTranslation() {
    return translation_;
  }

  glm::quat PointCloud::GetRotation() {
    return rotation_;
  }

  glm::vec3 PointCloud::GetTranslationFTFSM() {
    return translation_ftfsm_;
  }

  glm::quat PointCloud::GetRotationFTFSM() {
    return rotation_ftfsm_;
  }

  glm::vec3 PointCloud::GetTranslationMFSM() {
    return translation_mfsm_;
  }

  glm::quat PointCloud::GetRotationMFSM() {
    return rotation_mfsm_;
  }

  std::vector<float> PointCloud::GetXYZValuesTSS() {
    return xyz_values_ss_;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud::GetPointCloudTransformed() {
    return cloud_transformed_;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud::GetPointCloud() {
    return cloud_;
  }

}