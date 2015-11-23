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

#include <stdio.h>
#include <string.h>
#include <vector>
#include <inttypes.h>
#include <float.h>
#include <cmath>
#include <cfenv>
#include <climits>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "tango-gl/conversions.h"
#include "tango-gl/camera.h"
#include "rgb-depth-sync/range_image.h"
#include "rgb-depth-sync/texture_drawable.h"
#include "rgb-depth-sync/point_cloud_data.h"
#include "rgb-depth-sync/shader.h"

namespace rgb_depth_sync {

  RangeImage::RangeImage() {
    texture_drawable_ = new rgb_depth_sync::TextureDrawable(
        shader::kRangeImageVertex, shader::kRangeImageFragment);
  }

  RangeImage::~RangeImage() { }

  void RangeImage::RenderDepthMap(glm::mat4 &color_t1_T_depth_t0,
                                  glm::mat4 &start_service_T_color_t1,
                                  const std::vector <float> &render_point_cloud_buffer,
                                  const std::vector <uint8_t> &rgb_buffer,
                                  const std::vector <uint32_t> &rgb_pcd_buffer,
                                  double &timestamp) {

    int depth_image_width = rgb_camera_intrinsics_.width;
    int depth_image_height = rgb_camera_intrinsics_.height;
    int depth_image_size = depth_image_width * depth_image_height;

    display_buffer_.resize(depth_image_size);
    std::fill(display_buffer_.begin(), display_buffer_.end(), 0);

    size_t point_cloud_size = render_point_cloud_buffer.size();

    transformed_unordered_point_cloud_to_image_frame_.clear();

    for (size_t i = 0; i < point_cloud_size-3; i = i+3) {
      float x = render_point_cloud_buffer[i];
      float y = render_point_cloud_buffer[i + 1];
      float z = render_point_cloud_buffer[i + 2];

      // depth_t0_point is the point in depth camera frame on timestamp t0.
      // (depth image timestamp).
      glm::vec4 depth_t0_point = glm::vec4(x, y, z, 1.0);

      // color_t1_point is the point in camera frame on timestamp t1.
      // (color image timestamp).
      glm::vec4 color_t1_point = color_t1_T_depth_t0 * depth_t0_point;

      int pixel_x, pixel_y;
      // get the coordinate on image plane.
      pixel_x = static_cast<int>((rgb_camera_intrinsics_.fx) *
                                 (color_t1_point.x / color_t1_point.z) +
                                 rgb_camera_intrinsics_.cx);

      pixel_y = static_cast<int>((rgb_camera_intrinsics_.fy) *
                                 (color_t1_point.y / color_t1_point.z) +
                                 rgb_camera_intrinsics_.cy);

      if (pixel_x > depth_image_width || pixel_y > depth_image_height || pixel_x < 0 ||
          pixel_y < 0) {
        continue;
      }

      int index = (pixel_x + pixel_y * rgb_camera_intrinsics_.width);

      // (pixel depth - near limit) / (far limit - near limit)
      float depth_value = color_t1_point.z;
      uint8_t grayscale_value = (color_t1_point.z * kMeterToMillimeter) * UCHAR_MAX / kMaxDepthDistance;

      UpSampleGrayPixel(pixel_x, pixel_y, grayscale_value, &display_buffer_);

      // save rgb point cloud
      if (index >= rgb_pcd_buffer.size()) {
        LOGE("BAAAAAAAD INDEX: %d, size: %d", index, rgb_pcd_buffer.size());
      } else {
        transformed_unordered_point_cloud_to_image_frame_.push_back(color_t1_point.x);
        transformed_unordered_point_cloud_to_image_frame_.push_back(color_t1_point.y);
        transformed_unordered_point_cloud_to_image_frame_.push_back(color_t1_point.z);

        uint32_t tmp = rgb_pcd_buffer[index];
        // Due to historical reasons (PCL was first developed as a ROS package), the
        // RGB information is packed into an integer and casted to a float.
        transformed_unordered_point_cloud_to_image_frame_.push_back(*reinterpret_cast<float *>(&tmp));
      }
    }

    texture_drawable_->SetGrayTexture(display_buffer_.data(),
                                      rgb_camera_intrinsics_.width,
                                      rgb_camera_intrinsics_.height);
  }

  void RangeImage::RenderRGBMap(glm::mat4 &color_t1_T_depth_t0,
                                glm::mat4 &start_service_T_color_t1,
                                const std::vector <float> &render_point_cloud_buffer,
                                const std::vector <uint8_t> &rgb_buffer,
                                const std::vector <uint32_t> &rgb_pcd_buffer,
                                double &timestamp) {

    int depth_image_width = rgb_camera_intrinsics_.width;
    int depth_image_height = rgb_camera_intrinsics_.height;
    int depth_image_size = depth_image_width * depth_image_height;

    display_buffer_.resize(depth_image_size*3);
    std::fill(display_buffer_.begin(), display_buffer_.end(), 0);

    size_t point_cloud_size = render_point_cloud_buffer.size();

    transformed_unordered_point_cloud_to_image_frame_.clear();

    for (size_t i = 0; i < point_cloud_size-3; i = i+3) {
      float x = render_point_cloud_buffer[i];
      float y = render_point_cloud_buffer[i + 1];
      float z = render_point_cloud_buffer[i + 2];

      // depth_t0_point is the point in depth camera frame on timestamp t0.
      // (depth image timestamp).
      glm::vec4 depth_t0_point = glm::vec4(x, y, z, 1.0);

      // color_t1_point is the point in camera frame on timestamp t1.
      // (color image timestamp).
      glm::vec4 color_t1_point = color_t1_T_depth_t0 * depth_t0_point;

      int pixel_x, pixel_y;
      // get the coordinate on image plane.
      pixel_x = static_cast<int>((rgb_camera_intrinsics_.fx) *
                                 (color_t1_point.x / color_t1_point.z) +
                                 rgb_camera_intrinsics_.cx);

      pixel_y = static_cast<int>((rgb_camera_intrinsics_.fy) *
                                 (color_t1_point.y / color_t1_point.z) +
                                 rgb_camera_intrinsics_.cy);

      if (pixel_x > depth_image_width || pixel_y > depth_image_height || pixel_x < 0 ||
                                                                         pixel_y < 0) {
        continue;
      }

      int index = (pixel_x + pixel_y * rgb_camera_intrinsics_.width);

      UpSampleRGBPixel(pixel_x, pixel_y,
                    rgb_buffer[index * 3],
                    rgb_buffer[index * 3 + 1],
                    rgb_buffer[index * 3 + 2], &display_buffer_);

      // save rgb point cloud
      if (index >= rgb_pcd_buffer.size()) {
        LOGE("BAAAAAAAD INDEX: %d, size: %d", index, rgb_pcd_buffer.size());
      } else {
        transformed_unordered_point_cloud_to_image_frame_.push_back(color_t1_point.x);
        transformed_unordered_point_cloud_to_image_frame_.push_back(color_t1_point.y);
        transformed_unordered_point_cloud_to_image_frame_.push_back(color_t1_point.z);

        uint32_t tmp = rgb_pcd_buffer[index];
        // Due to historical reasons (PCL was first developed as a ROS package), the
        // RGB information is packed into an integer and casted to a float.
        transformed_unordered_point_cloud_to_image_frame_.push_back(*reinterpret_cast<float *>(&tmp));
      }
    }

    texture_drawable_->SetRGBTexture(display_buffer_.data(),
                                    rgb_camera_intrinsics_.width,
                                    rgb_camera_intrinsics_.height);
  }

  void RangeImage::UpSampleRGBPixel(int pixel_x, int pixel_y,
                               uint8_t r, uint8_t g, uint8_t b,
                               std::vector<uint8_t> *display_buffer) {

    int image_size = rgb_camera_intrinsics_.width * rgb_camera_intrinsics_.height;

    // upsample depth pixel to image frame
    for (int x = -kWindowSize; x <= kWindowSize; ++x) {
      for (int y = -kWindowSize; y <= kWindowSize; ++y) {

        int index = ((pixel_x + x) + (pixel_y + y) * rgb_camera_intrinsics_.width);

        if (index > 0 && index < image_size) {
          (*display_buffer)[index * 3] = r;
          (*display_buffer)[index * 3 + 1] = g;
          (*display_buffer)[index * 3 + 2] = b;
        }
      }
    }
  }

  void RangeImage::UpSampleGrayPixel(int pixel_x, int pixel_y,
                                     uint8_t gray_value,
                                     std::vector<uint8_t> *display_buffer) {

    int image_size = rgb_camera_intrinsics_.width * rgb_camera_intrinsics_.height;

    // upsample depth pixel to image frame
    for (int x = -kWindowSize; x <= kWindowSize; ++x) {
      for (int y = -kWindowSize; y <= kWindowSize; ++y) {

        int index = ((pixel_x + x) + (pixel_y + y) * rgb_camera_intrinsics_.width);

        if (index > 0 && index < image_size) {
          (*display_buffer)[index] = gray_value;
        }
      }
    }
  }

  const std::vector<float>& RangeImage::GetRGBPointCloud() const {
    return transformed_unordered_point_cloud_to_image_frame_;
  }

  void RangeImage::SetCameraIntrinsics(TangoCameraIntrinsics intrinsics) {
    rgb_camera_intrinsics_ = intrinsics;
    const float kNearClip = 0.1;
    const float kFarClip = 10.0;
    projection_matrix_ar_ = tango_gl::Camera::ProjectionMatrixForCameraIntrinsics(
        intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy,
        intrinsics.cx, intrinsics.cy, kNearClip, kFarClip);
  }

  void RangeImage::Draw(int width, int height) {

    if (height == 0 || width == 0) {
      LOGE("The Scene received an invalid height of 0 in SetupViewPort.");
    }
    texture_drawable_->RenderImage(width, height);
  }

}  // namespace rgb_depth_sync
