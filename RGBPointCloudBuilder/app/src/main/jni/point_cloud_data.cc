#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include "rgb-depth-sync/point_cloud_data.h"
#include "rgb-depth-sync/tcp_client.h"

namespace rgb_depth_sync {

  PointCloudData::PointCloudData(int id) {
    id_ = id;
  }

  PointCloudData::~PointCloudData() {
    LOGE("PointCloudData is destroyed...");
  }

  void PointCloudData::SetPCDWithRGBData(const std::vector<float> pcd) {
    pcd_with_rgb_data_.clear();
    xyz_T_color_buffer_.clear();
    rgb_values_.clear();

    for (int i = 0; i < pcd.size(); i=i+4) {
      xyz_T_color_buffer_.push_back(pcd[i]);
      xyz_T_color_buffer_.push_back(pcd[i+1]);
      xyz_T_color_buffer_.push_back(pcd[i+2]);
      rgb_values_.push_back(pcd[i+3]);
      pcd_with_rgb_data_.push_back(pcd[i]);
      pcd_with_rgb_data_.push_back(pcd[i+1]);
      pcd_with_rgb_data_.push_back(pcd[i+2]);
      pcd_with_rgb_data_.push_back(pcd[i+3]);
    }
  }

  void PointCloudData::SetPCDData(const std::vector<float> pcd) {
    xyz_T_color_buffer_ = pcd;
  }

  void PointCloudData::SetTranslation(const std::vector<float> translation) {
    translation_.x = translation[0];
    translation_.y = translation[1];
    translation_.z = translation[2];
  }

  void PointCloudData::SetRotation(const std::vector<float> rotation) {
    rotation_.w = rotation[0]; //w
    rotation_.x = rotation[1]; //x
    rotation_.y = rotation[2]; //y
    rotation_.z = rotation[3]; //z
  }

  void PointCloudData::SetPose(const glm::mat4 pose) {
    ss_T_color_ = pose;
  }

  void PointCloudData::MapXYZWithRGB(glm::mat4 &ss_T_color,
                                     glm::mat4 &ss_T_depth,
                                     glm::mat4 &color_T_depth,
                                     glm::mat4 &opengl_T_depth,
                                     const std::vector<float> &render_point_cloud_buffer,
                                     const std::vector<uint8_t> &rgb_buffer) {

    TangoCameraIntrinsics depth_camera_intrinsics;

    TangoErrorType ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_DEPTH, &depth_camera_intrinsics);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to get the intrinsics for the color camera.");
    }

    //range_image_.create(720, 1280, cv::CV_32FC1);
    color_T_depth_ = color_T_depth;
    opengl_T_depth_ = opengl_T_depth;
    ss_T_color_ = ss_T_color;
    ss_T_depth_ = ss_T_depth;

    translation_ = util::GetTranslationFromMatrix(ss_T_color);
    rotation_ = util::GetRotationFromMatrix(ss_T_color);

    int depth_image_width = rgb_camera_intrinsics_.width;
    int depth_image_height = rgb_camera_intrinsics_.height;
    int depth_image_size = depth_image_width * depth_image_height;

    size_t point_cloud_size = render_point_cloud_buffer.size();

    pcd_with_rgb_data_.clear();
    xyz_T_color_buffer_.clear();
    rgb_values_.clear();
    xyz_buffer_.clear();
    xyz_buffer_ = render_point_cloud_buffer;
    //range_image_.clear();
    //range_image_.resize(720*1280);
    //std::fill(range_image_.begin(), range_image_.end(), -1);
    //cv_vertrices_.clear();

    //glm::vec3 translation = util::GetTranslationFromMatrix(ss_T_color_);
    //glm::quat rotation = util::GetRotationFromMatrix(ss_T_color_);

    for (size_t i = 0; i < point_cloud_size-3; i = i+3) {
      float x = render_point_cloud_buffer[i];
      float y = render_point_cloud_buffer[i + 1];
      float z = render_point_cloud_buffer[i + 2];

      // without distortion

      if (z > 1.25f || z < 0.5f) {
        continue;
      }

      int pixel_x, pixel_y;
      double ru = sqrt((x*x + y*y) / z*z);
      double rd = ru + depth_camera_intrinsics.distortion[0] * pow(ru,3) + depth_camera_intrinsics.distortion[0] * pow(ru,5) + depth_camera_intrinsics.distortion[0] * pow(ru,7);

      // get the coordinate on image plane.
      pixel_x = static_cast<int>(x / z * depth_camera_intrinsics.fx * rd / ru + depth_camera_intrinsics.cx);
      pixel_y = static_cast<int>(y / z * depth_camera_intrinsics.fy * rd / ru + depth_camera_intrinsics.cy);

      x = static_cast<float>(z * (pixel_x - depth_camera_intrinsics.cx) / depth_camera_intrinsics.fx);
      y = static_cast<float>(z * (pixel_y - depth_camera_intrinsics.cy) / depth_camera_intrinsics.fy);

      // transform depth point to color frame
      glm::vec3 opengl_point = glm::vec3(opengl_T_depth * glm::vec4(x, y, z, 1.0));
      glm::vec3 color_point = glm::vec3(color_T_depth * glm::vec4(x, y, z, 1.0));

      // normalized radial distance
      ru = sqrt((color_point.x*color_point.x + color_point.y*color_point.y) / color_point.z*color_point.z);
      rd = ru + rgb_camera_intrinsics_.distortion[0] * pow(ru,3) + rgb_camera_intrinsics_.distortion[0] * pow(ru,5) + rgb_camera_intrinsics_.distortion[0] * pow(ru,7);

      // get the coordinate on image plane.
      pixel_x = static_cast<int>(color_point.x / color_point.z * rgb_camera_intrinsics_.fx * rd / ru + rgb_camera_intrinsics_.cx);

      pixel_y = static_cast<int>(color_point.y / color_point.z * rgb_camera_intrinsics_.fy * rd / ru + rgb_camera_intrinsics_.cy);

      /*pixel_x = static_cast<int>(color_point.x / color_point.z * rgb_camera_intrinsics_.fx + rgb_camera_intrinsics_.cx);

      pixel_y = static_cast<int>(color_point.y / color_point.z * rgb_camera_intrinsics_.fy + rgb_camera_intrinsics_.cy);*/


      if (pixel_x > depth_image_width || pixel_y > depth_image_height || pixel_x < 0 ||
          pixel_y < 0) {
        continue;
      }

      size_t index = (pixel_x + pixel_y * rgb_camera_intrinsics_.width);

      // save rgb point cloud
      if (index >= rgb_buffer.size()) {
        LOGE("BAAAAAAAD INDEX: %d, size: %d", rgb_buffer.size());
      } else {
        /*int image_size = rgb_camera_intrinsics_.width * rgb_camera_intrinsics_.height;
        for (int x = -10; x <= 10; ++x) {
          for (int y = -10; y <= 10; ++y) {
            int index = ((pixel_x + x) + (pixel_y + y) * rgb_camera_intrinsics_.width);
            if (index > 0 && index < image_size) {
              range_image_[index] = color_point.z;
            }
          }
        }*/

        pcd_with_rgb_data_.push_back(color_point.x);
        pcd_with_rgb_data_.push_back(color_point.y);
        pcd_with_rgb_data_.push_back(color_point.z);

        xyz_T_color_buffer_.push_back(opengl_point.x);
        xyz_T_color_buffer_.push_back(opengl_point.y);
        xyz_T_color_buffer_.push_back(opengl_point.z);

        //cv_vertrices_.push_back(cv::Point3f(opengl_point.x, opengl_point.z, opengl_point.z));

        rgb_values_.push_back(rgb_buffer[index*3]);
        rgb_values_.push_back(rgb_buffer[index*3 + 1]);
        rgb_values_.push_back(rgb_buffer[index*3 + 2]);

        // Due to historical reasons (PCL was first developed as a ROS package), the
        // RGB information is packed into an integer and casted to a float.
        uint32_t rgb = ((uint32_t) (rgb_buffer[index*3])) << 16 | ((uint32_t) (rgb_buffer[index*3 + 1])) << 8 | (uint32_t) (rgb_buffer[index*3 + 2]);
        pcd_with_rgb_data_.push_back(*reinterpret_cast<float *>(&rgb));
      }
    }
    //LOGE("PointCloudData : MapXYZWithRGB stop...");
  }

  /*void PointCloudData::MapXYZWithRGB(TangoPoseData &ss_T_color,
                                     glm::mat4 &color_T_depth,
                                     glm::mat4 &opengl_T_depth,
                                     const std::vector <float> &render_point_cloud_buffer,
                                     const std::vector <uint8_t> &rgb_map_buffer,
                                     const std::vector <uint32_t> &rgb_pcd_buffer) {

    translation_[0] = ss_T_color.translation[0];
    translation_[1] = ss_T_color.translation[1];
    translation_[2] = ss_T_color.translation[2];

    rotation_[0] = ss_T_color.orientation[0];
    rotation_[1] = ss_T_color.orientation[1];
    rotation_[2] = ss_T_color.orientation[2];
    rotation_[3] = ss_T_color.orientation[3];

    int depth_image_width = rgb_camera_intrinsics_.width;
    int depth_image_height = rgb_camera_intrinsics_.height;
    int depth_image_size = depth_image_width * depth_image_height;

    size_t point_cloud_size = render_point_cloud_buffer.size();

    pcd_with_rgb_data_.clear();
    xyz_T_color_buffer_.clear();
    rgb_values_.clear();

    for (size_t i = 0; i < point_cloud_size-3; i = i+3) {
      float x = render_point_cloud_buffer[i];
      float y = render_point_cloud_buffer[i + 1];
      float z = render_point_cloud_buffer[i + 2];

      // transform depth point to color frame
      glm::vec3 color_point = glm::vec3(color_T_depth * glm::vec4(x, y, z, 1.0));
      glm::vec3 opengl_point = glm::vec3(opengl_T_depth * glm::vec4(x, y, z, 1.0));

      // normalized radial distance
      //float ru = Math.sqrt((x*x + y*y) / z*z);
      //float rd =

      int pixel_x, pixel_y;
      // get the coordinate on image plane.
      pixel_x = static_cast<int>((rgb_camera_intrinsics_.fx) *
                                 (color_point.x / color_point.z) +
                                 rgb_camera_intrinsics_.cx);

      pixel_y = static_cast<int>((rgb_camera_intrinsics_.fy) *
                                 (color_point.y / color_point.z) +
                                 rgb_camera_intrinsics_.cy);

      if (pixel_x > depth_image_width || pixel_y > depth_image_height || pixel_x < 0 ||
          pixel_y < 0) {
        continue;
      }

      size_t index = (pixel_x + pixel_y * rgb_camera_intrinsics_.width);

      // save rgb point cloud
      if (index >= rgb_pcd_buffer.size()) {
        LOGE("BAAAAAAAD INDEX: %d, size: %d", index, rgb_pcd_buffer.size());
      } else {
        pcd_with_rgb_data_.push_back(color_point.x);
        pcd_with_rgb_data_.push_back(color_point.y);
        pcd_with_rgb_data_.push_back(color_point.z);

        xyz_T_color_buffer_.push_back(opengl_point.x);
        xyz_T_color_buffer_.push_back(opengl_point.y);
        xyz_T_color_buffer_.push_back(opengl_point.z);

        rgb_values_.push_back(rgb_map_buffer[index*3]);
        rgb_values_.push_back(rgb_map_buffer[index*3 + 1]);
        rgb_values_.push_back(rgb_map_buffer[index*3 + 2]);

        uint32_t tmp = rgb_pcd_buffer[index];
        // Due to historical reasons (PCL was first developed as a ROS package), the
        // RGB information is packed into an integer and casted to a float.
        pcd_with_rgb_data_.push_back(*reinterpret_cast<float *>(&tmp));
        rgb_values_pcd_file_.push_back(*reinterpret_cast<float *>(&tmp));
      }
    }
  }*/

  void PointCloudData::SetTranslation(const glm::vec3 translation) {
    translation_ = translation;
  }

  void PointCloudData::SetRotation(const glm::quat rotation) {
    rotation_ = rotation;
  }

  glm::vec3 PointCloudData::GetTranslation() {
    return translation_;
  }

  glm::quat PointCloudData::GetRotation() {
    return rotation_;
  }

  std::vector<float> PointCloudData::GetRGBPCDFileValues() {
    return rgb_values_pcd_file_;
  }

  std::vector<uint8_t> PointCloudData::GetRGBValues() {
    return rgb_values_;
  }

  std::vector<float> PointCloudData::GetXYZValues_T_Color() {
    return xyz_T_color_buffer_;
  }

  std::vector<float> PointCloudData::GetXYZValues() {
    return xyz_buffer_;
  }

  std::vector<float> PointCloudData::GetPCDData() {
    //LOGE("PointCloudData : GetPCDData");
    return pcd_with_rgb_data_;
  }

  int PointCloudData::GetId() {
    return id_;
  }

  void PointCloudData::SetCameraIntrinsics(TangoCameraIntrinsics intrinsics) {
    rgb_camera_intrinsics_ = intrinsics;
    /*const float kNearClip = 0.1;
    const float kFarClip = 10.0;
    projection_matrix_ar_ = tango_gl::Camera::ProjectionMatrixForCameraIntrinsics(
        intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy,
        intrinsics.cx, intrinsics.cy, kNearClip, kFarClip);*/
  }

  std::vector<float> PointCloudData::GetRangeImage() {
    return range_image_;
  }

} // namespace rgb_depth_sync