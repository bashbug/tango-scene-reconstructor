#include <tango_client_api.h>
#include <tango-gl/util.h>
#include <tango-gl/conversions.h>

#include "rgb-depth-sync/util.h"

#ifndef RGBPOINTCLOUDBUILDER_CONVERSION_H
#define RGBPOINTCLOUDBUILDER_CONVERSION_H

namespace rgb_depth_sync {

  class Conversion {
    public:
      static Conversion* GetInstance();
      glm::mat4 XYZ_T_RGB(const double &rgb_timestamp, const double &xyz_timestamp);
      glm::mat4 OpenGL_T_RGB(const glm::mat4 &ss_T_rgb);
      glm::mat4 OpenGL_T_OpenGLCamera(const glm::mat4 &ss_T_rgb);
      glm::mat4 SS_T_RGB(const double &rgb_timestamp);
      glm::mat4 OpenGL_T_Device(const glm::mat4 &ss_T_device);
      glm::mat4 OpenGL_T_device_T_OpenGLCamera(const glm::mat4 &ss_T_device);
      glm::mat4 device_T_color_;
      glm::mat4 device_T_depth_;
      glm::mat4 color_T_device_;
    private:
      Conversion() {};
      Conversion(Conversion const&) {};
      void operator = (Conversion const&) {};
      void SetMatrices();
  };
}

#endif //RGBPOINTCLOUDBUILDER_CONVERSION_H
