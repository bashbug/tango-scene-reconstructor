//
// Created by anastasia on 06.01.16.
//

#ifndef RGBPOINTCLOUDBUILDER_XYZ_RGB_H
#define RGBPOINTCLOUDBUILDER_XYZ_RGB_H

#include <tango-gl/util.h>
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  class XYZRGB {
    public:
      XYZRGB();
      ~XYZRGB();
      void SetYUVBuffer(std::vector<uint8_t> yuv_buffer, size_t width, size_t height);
      void SetXYZBuffer(std::vector<float> xyz_buffer);
      void SetColor_T_DepthTransformation(glm::mat4 &color_T_depth);
      void SetStartOfService_T_ColorTransformation(glm::mat4 &ss_T_color);
      void SetCameraIntrinsics(TangoCameraIntrinsics intrinsics);
      std::vector<uint8_t> GetRGBBuffer();
      std::vector<uint32_t> GetStuffedRGBBuffer();
      std::vector<float> GetXYZRGBPointCloud();

    private:
      void SetRGBBuffer();
      void MapXYZWithRGB();
      void YUV2RGB(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t* r,
                         uint8_t* g, uint8_t* b, uint32_t* rgb);
      size_t yuv_width_;
      size_t yuv_height_;
      size_t uv_buffer_offset_;
      size_t yuv_size_;
      std::vector<uint8_t> yuv_buffer_;
      std::vector<float> xyz_buffer_;
      std::vector<float> xyz_rgb_buffer_;
      std::vector<uint32_t> rgb_stuffed_buffer_;
      std::vector<uint8_t> r_g_b_buffer_;
      std::vector<uint32_t> rgb_stuffed_buffer_ordered_;
      std::vector<uint8_t> r_g_b_buffer_ordered_;

      glm::mat4 open_gl_T_color_;
      glm::mat4 color_T_depth_;
      TangoCameraIntrinsics rgb_camera_intrinsics_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_XYZ_RGB_H
