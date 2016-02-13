#include "rgb-depth-sync/pcd.h"

namespace rgb_depth_sync {

  PCD::PCD() {
    depth_image_pixels_.width = 320;
    depth_image_pixels_.height = 180;
    depth_image_pixels_.resize(320*180);
  }

  PCD::~PCD() {
    LOGE("PointCloudData is destroyed...");
  }

  void PCD::SetPCDWithRGBData(const std::vector<float> pcd) {
    pcd_with_rgb_data_.clear();
    xyz_values_.clear();
    rgb_values_.clear();

    for (int i = 0; i < pcd.size(); i=i+4) {
      xyz_values_.push_back(pcd[i]);
      xyz_values_.push_back(pcd[i+1]);
      xyz_values_.push_back(pcd[i+2]);
      rgb_values_.push_back(pcd[i+3]);
      pcd_with_rgb_data_.push_back(pcd[i]);
      pcd_with_rgb_data_.push_back(pcd[i+1]);
      pcd_with_rgb_data_.push_back(pcd[i+2]);
      pcd_with_rgb_data_.push_back(pcd[i+3]);
    }
  }

  void PCD::SetPCDData(const std::vector<float> pcd) {
    xyz_values_ = pcd;
  }

  void PCD::SetTranslation(const std::vector<float> translation) {
    translation_.x = translation[0];
    translation_.y = translation[1];
    translation_.z = translation[2];
  }

  void PCD::SetRotation(const std::vector<float> rotation) {
    rotation_.w = rotation[0]; //w
    rotation_.x = rotation[1]; //x
    rotation_.y = rotation[2]; //y
    rotation_.z = rotation[3]; //z
  }

  void PCD::SetPose(const glm::mat4 pose) {
    ss_T_color_ = pose;
  }

  void PCD::MapXYZWithRGB(const std::vector<float> &xyz, const std::vector<uint8_t> &rgb,
                          const double &xyz_timestamp, const double &rgb_timestamp) {

    TangoCameraIntrinsics depth_camera_intrinsics;
    TangoCameraIntrinsics color_camera_intrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_DEPTH, &depth_camera_intrinsics);
    TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &color_camera_intrinsics);

    Conversion *conversion = Conversion::GetInstance();
    color_T_depth_ = conversion->XYZ_T_RGB(rgb_timestamp, xyz_timestamp);
    ss_T_color_ = conversion->SS_T_RGB(rgb_timestamp);
    opengl_T_color_ = conversion->OpenGL_T_RGB(ss_T_color_);
    opengl_T_openglCamera_ = conversion->OpenGL_T_OpenGLCamera(ss_T_color_);

    translation_ = util::GetTranslationFromMatrix(ss_T_color_);
    rotation_ = util::GetRotationFromMatrix(ss_T_color_);

    size_t xyz_size = xyz.size();
    size_t rgb_size = rgb.size();

    for (size_t i = 0; i < xyz_size - 3; i = i + 3) {
      float x = xyz[i];
      float y = xyz[i + 1];
      float z = xyz[i + 2];

      if (z > 1.5 && z < 0.5) {
        continue;
      }

      int pixel_x, pixel_y, depth_pixel_x, depth_pixel_y;

      // transform depth point to color frame
      glm::vec3 color_point = glm::vec3(color_T_depth_ * glm::vec4(x, y, z, 1.0f));
      glm::vec3 opengl_point = glm::vec3(opengl_T_color_ * glm::vec4(x, y, z, 1.0f));

      pixel_x = static_cast<int>(opengl_point.x / opengl_point.z * color_camera_intrinsics.fx +
                                 color_camera_intrinsics.cx);

      pixel_y = static_cast<int>(opengl_point.y / opengl_point.z * color_camera_intrinsics.fy +
                                 color_camera_intrinsics.cy);

      depth_pixel_x = static_cast<int>(opengl_point.x / opengl_point.z * depth_camera_intrinsics.fx +
          depth_camera_intrinsics.cx);

      depth_pixel_y = static_cast<int>(opengl_point.y / opengl_point.z * depth_camera_intrinsics.fy +
                                       depth_camera_intrinsics.cy);


      size_t depth_index = depth_pixel_x + depth_pixel_y * depth_camera_intrinsics.width;

      ProjectiveImage::Point p(opengl_point.x, opengl_point.y, opengl_point.z, depth_index, -1.0f, false);

      if (depth_pixel_x >= 0 && depth_pixel_x < depth_camera_intrinsics.width && depth_pixel_y >= 0 && depth_pixel_y < depth_camera_intrinsics.height) {
        depth_image_pixels_.getPixelNoCheck(depth_pixel_x, depth_pixel_y).push_back(p);
      }

      size_t index = pixel_x + pixel_y * color_camera_intrinsics.width;

      // save rgb point cloud
      if (index * 3 + 2 >= rgb.size() || pixel_x < 0 && pixel_x >= color_camera_intrinsics.width || pixel_y < 0 && pixel_y >= color_camera_intrinsics.height) {
        //break;
        continue;
        //LOGE("COLOR BAAAAAAAD INDEX: %d, size: %d", index, rgb.size());
      } else {

        pcd_with_rgb_data_.push_back(opengl_point.x);
        pcd_with_rgb_data_.push_back(opengl_point.y);
        pcd_with_rgb_data_.push_back(opengl_point.z);

        xyz_values_.push_back(opengl_point.x);
        xyz_values_.push_back(opengl_point.y);
        xyz_values_.push_back(opengl_point.z);

        rgb_values_.push_back(rgb[index * 3]);
        rgb_values_.push_back(rgb[index * 3 + 1]);
        rgb_values_.push_back(rgb[index * 3 + 2]);

        // Due to historical reasons (PCL was first developed as a ROS package), the
        // RGB information is packed into an integer and casted to a float.
        uint32_t rgb_tmp =
            ((uint32_t)(rgb[index * 3])) << 16 | ((uint32_t)(rgb[index * 3 + 1])) << 8 |
            (uint32_t)(rgb[index * 3 + 2]);
        pcd_with_rgb_data_.push_back(*reinterpret_cast<float *>(&rgb_tmp));
      }
    }
  }

  void PCD::SetKeyPointsAndDescriptors(const std::vector<cv::KeyPoint>& frame_key_points, cv::Mat frame_descriptors) {
    frame_key_points_ = frame_key_points;
    frame_descriptors_ = frame_descriptors;
  }

  void PCD::SetFrame(const cv::Mat& frame) {
    frame_ = frame;
  }

  void PCD::SetTranslation(const glm::vec3 translation) {
    translation_ = translation;
  }

  void PCD::SetRotation(const glm::quat rotation) {
    rotation_ = rotation;
  }

  glm::vec3 PCD::GetTranslation() {
    return translation_;
  }

  glm::quat PCD::GetRotation() {
    return rotation_;
  }

  std::vector<float> PCD::GetRGBPCDFileValues() {
    return rgb_values_pcd_file_;
  }

  std::vector<uint8_t> PCD::GetRGBValues() {
    return rgb_values_;
  }

  std::vector<float> PCD::GetXYZValues() {
    return xyz_values_;
  }

  std::vector<float> PCD::GetPCDData() {
    //LOGE("PointCloudData : GetPCDData");
    return pcd_with_rgb_data_;
  }

  ProjectiveImage::ImagePixels PCD::GetImagePixels() {
    return depth_image_pixels_;
  }

} // namespace rgb_depth_sync