/*
 * Class writes rgb point cloud either to an file or
 * sends it via TCP socket.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H

#include <math.h>
#include <string>
#include <vector>

#include <tango-gl/util.h>

#include <opencv2/opencv.hpp>

#include "rgb-depth-sync/tcp_client.h"
#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/conversion.h"

namespace rgb_depth_sync {

  class PCD {
    public:
      PCD();
      ~PCD();
      void MapXYZWithRGB(const std::vector<float> &xyz, const std::vector<uint8_t> &rgb,
                         const double &xyz_timestamp, const double &rgb_timestamp);
      void SetPCDData(const std::vector<float> pcd);
      void SetPCDWithRGBData(const std::vector<float> pcd);
      void SetTranslation(const std::vector<float> translation);
      void SetRotation(const std::vector<float> rotation);
      void SetTranslation(const glm::vec3 translation);
      void SetRotation(const glm::quat rotation);
      void SetPose(const glm::mat4 pose);
      std::vector<float> GetPCDData();
      std::vector<float> GetPCDWithRGBData();
      glm::vec3 GetTranslation();
      glm::quat GetRotation();
      std::vector<float> GetXYZValues();
      std::vector<uint8_t> GetRGBValues();
      std::vector<float> GetRGBPCDFileValues();
      glm::mat4 GetPose() { return ss_T_color_; }
      glm::mat4 GetOpenGL_T_OpenGLCameraPose() { return opengl_T_openglCamera_; }
      glm::mat4 GetOpenGL_T_RGB() {return  opengl_T_color_;}

    private:
      std::vector<float> pcd_with_rgb_data_;
      std::vector<float> xyz_values_;
      std::vector<uint8_t> rgb_values_;
      std::vector<float> rgb_values_pcd_file_;
      glm::vec3 translation_;
      glm::quat rotation_;
      glm::mat4 color_T_depth_;
      glm::mat4 ss_T_color_;
      glm::mat4 opengl_T_color_;
      glm::mat4 opengl_T_openglCamera_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
