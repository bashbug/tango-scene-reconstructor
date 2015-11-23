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

#ifndef RGB_DEPTH_SYNC_RANGE_IMAGE_H_
#define RGB_DEPTH_SYNC_RANGE_IMAGE_H_

#include <thread>
#include <mutex>
#include "tango_client_api.h"
#include "tango-gl/util.h"
#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/texture_drawable.h"

namespace rgb_depth_sync {

  class RangeImage {
    public:
      explicit RangeImage();
      ~RangeImage();
      void RenderDepthMap(glm::mat4 &color_t1_T_depth_t0,
                          glm::mat4 &start_service_T_color_t1,
                          const std::vector <float> &render_point_cloud_buffer,
                          const std::vector <uint8_t> &rgb_buffer,
                          const std::vector <uint32_t> &rgb_pcd_buffer,
                          double &timestamp);
      void RenderRGBMap(glm::mat4& color_t1_T_depth_t0,
                        glm::mat4 &start_service_T_color_t1,
                        const std::vector<float> &render_point_cloud_buffer,
                        const std::vector<uint8_t> &rgb_buffer,
                        const std::vector <uint32_t> &rgb_pcd_buffer,
                        double &timestamp);
      // Make the rgb point cloud available for later processes
      const std::vector<float>& GetRGBPointCloud() const;
      void SetCameraIntrinsics(TangoCameraIntrinsics intrinsics);
      void Draw(int width, int height);
    private:
      // pixel upsample in -2 to 2 in x and y direction
      static const int kWindowSize = 2;
      // The defined max distance for a depth value to 8 meters.
      static const int kMaxDepthDistance = 8000;
      // The meter to millimeter conversion.
      static const int kMeterToMillimeter = 1000;
      std::vector<float> rgb_pcd_buffer_;
      std::vector<uint8_t> display_buffer_;
      void UpSampleRGBPixel(int pixel_x, int pixel_y,
                            uint8_t r, uint8_t g, uint8_t b,
                            std::vector<uint8_t> *rgb_display_buffer);
      void UpSampleGrayPixel(int pixel_x, int pixel_y,
                             uint8_t gray_value,
                             std::vector<uint8_t> *gray_display_buffer);
      // The camera intrinsics of current device. Note that the color camera and
      // depth camera are the same hardware on the device.
      TangoCameraIntrinsics rgb_camera_intrinsics_;
      // Transform between Color camera and Depth Camera.
      glm::mat4 depth_camera_T_color_camera_;
      glm::mat4 projection_matrix_ar_;
      // Draw either an rgb or a gray range image to the color camera frame
      TextureDrawable* texture_drawable_;
      // Store the transformed rgb point cloud for later processes
      std::vector<float> transformed_unordered_point_cloud_to_image_frame_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_RANGE_IMAGE_H_
