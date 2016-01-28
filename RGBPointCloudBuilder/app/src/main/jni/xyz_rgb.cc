//
// Created by anastasia on 06.01.16.
//
#include <tango-gl/conversions.h>
#include "rgb-depth-sync/xyz_rgb.h"

namespace rgb_depth_sync {

  XYZRGB::XYZRGB() { }

  XYZRGB::~XYZRGB() { }

  void XYZRGB::SetYUVBuffer(std::vector<uint8_t> yuv_buffer, size_t width, size_t height) {
    yuv_buffer_.clear();
    yuv_buffer_ = yuv_buffer;
    yuv_width_ = width;
    yuv_height_ = height;
    uv_buffer_offset_ = width * height;
    yuv_size_ = width * height + width * height / 2;

    SetRGBBuffer();
  }

  void XYZRGB::SetRGBBuffer() {
    r_g_b_buffer_.clear();
    rgb_stuffed_buffer_.clear();
    r_g_b_buffer_.resize(yuv_width_ * yuv_height_ * 3);
    rgb_stuffed_buffer_.resize(yuv_width_ * yuv_height_);

    for (size_t i = 0; i < yuv_height_; i++) {
      for (size_t j = 0; j < yuv_width_; j++) {
        size_t x_index = j;
        if (j % 2 != 0) {
          x_index = j - 1;
        }

        size_t rgb_index = (i * yuv_width_ + j) * 3;

        // The YUV texture format is NV21,
        // yuv_buffer_ buffer layout:
        //   [y0, y1, y2, ..., yn, v0, u0, v1, u1, ..., v(n/4), u(n/4)]
        YUV2RGB(yuv_buffer_[i * yuv_width_ + j],
                yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index + 1],
                yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index],
                &r_g_b_buffer_[rgb_index],
                &r_g_b_buffer_[rgb_index + 1],
                &r_g_b_buffer_[rgb_index + 2],
                &rgb_stuffed_buffer_[i * yuv_width_ + j]);
      }
    }
  }

  void XYZRGB::SetXYZBuffer(std::vector<float> xyz_buffer) {
    xyz_buffer_ = xyz_buffer;
  }

  void XYZRGB::SetColor_T_DepthTransformation(glm::mat4 &color_T_depth) {
    color_T_depth_ = color_T_depth;
  }

  void XYZRGB::SetStartOfService_T_ColorTransformation(glm::mat4 &ss_T_color) {
    open_gl_T_color_ = tango_gl::conversions::opengl_world_T_tango_world() * ss_T_color;
  }

  void XYZRGB::SetCameraIntrinsics(TangoCameraIntrinsics intrinsics) {
    rgb_camera_intrinsics_ = intrinsics;
  }

  std::vector<uint8_t> XYZRGB::GetRGBBuffer() {
    return r_g_b_buffer_ordered_;
  }

  std::vector<uint32_t> XYZRGB::GetStuffedRGBBuffer() {
    return rgb_stuffed_buffer_ordered_;
  }

  std::vector<float> XYZRGB::GetXYZRGBPointCloud() {
    MapXYZWithRGB();
    return xyz_rgb_buffer_;
  }

  void XYZRGB::MapXYZWithRGB() {
    r_g_b_buffer_ordered_.clear();
    rgb_stuffed_buffer_ordered_.clear();
    xyz_rgb_buffer_.clear();

    int depth_image_width = rgb_camera_intrinsics_.width;
    int depth_image_height = rgb_camera_intrinsics_.height;
    int depth_image_size = depth_image_width * depth_image_height;

    size_t point_cloud_size = xyz_buffer_.size();

    std::vector<float> tmp_xyz_buffer;
    tmp_xyz_buffer.resize(point_cloud_size);

    for (size_t i = 0; i < point_cloud_size-3; i = i+3) {
      float x = xyz_buffer_[i];
      float y = xyz_buffer_[i + 1];
      float z = xyz_buffer_[i + 2];

      // transform depth point to color frame
      glm::vec3 color_point = glm::vec3(color_T_depth_ * glm::vec4(x, y, z, 1.0));
      glm::vec3 opengl_point = glm::vec3(open_gl_T_color_ * glm::vec4(x, y, z, 1.0));

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
      if (index >= rgb_stuffed_buffer_.size()) {
        LOGE("BAAAAAAAD INDEX: %d, size: %d", index, rgb_stuffed_buffer_.size());
      } else {
        xyz_rgb_buffer_.push_back(color_point.x);
        xyz_rgb_buffer_.push_back(color_point.y);
        xyz_rgb_buffer_.push_back(color_point.z);

        tmp_xyz_buffer.push_back(opengl_point.x);
        tmp_xyz_buffer.push_back(opengl_point.y);
        tmp_xyz_buffer.push_back(opengl_point.z);

        uint32_t tmp = rgb_stuffed_buffer_[index];

        // Due to historical reasons (PCL was first developed as a ROS package), the
        // RGB information is packed into an integer and casted to a float.
        xyz_rgb_buffer_.push_back(*reinterpret_cast<float *>(&tmp));
        rgb_stuffed_buffer_ordered_.push_back(*reinterpret_cast<float *>(&tmp));

        r_g_b_buffer_ordered_.push_back(r_g_b_buffer_[index*3]);
        r_g_b_buffer_ordered_.push_back(r_g_b_buffer_[index*3 + 1]);
        r_g_b_buffer_ordered_.push_back(r_g_b_buffer_[index*3 + 2]);
      }


      xyz_buffer_.clear();
      xyz_buffer_ = tmp_xyz_buffer;

      /*LOGE("size xyz_buffer_ : %i", xyz_buffer_.size());
      LOGE("size xyz_rgb_buffer_ : %i", xyz_rgb_buffer_.size());
      LOGE("size r_g_b_buffer_ordered_ : %i", r_g_b_buffer_ordered_.size());
      LOGE("size rgb_stuffed_buffer_ordered_ : %i", rgb_stuffed_buffer_ordered_.size());
      LOGE("size tmp_xyz_buffer : %i", tmp_xyz_buffer.size());*/
    }
  }

  void XYZRGB::YUV2RGB(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t* r,
                       uint8_t* g, uint8_t* b, uint32_t* rgb) {
    *r = yValue + (1.370705f * (vValue - 128.0f));
    *g = yValue - (0.698001f * (vValue - 128.0f)) - (0.337633f * (uValue - 128.0f));
    *b = yValue + (1.732446f * (uValue - 128.0f));

    *rgb = ((uint32_t) (*r)) << 16 | ((uint32_t) (*g)) << 8 | (uint32_t) (*b);
  }

}
