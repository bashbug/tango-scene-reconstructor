/**
 * Singelton helper class which is created one time and is accessible globaly.
 * It holds the depth and color camera intrinsics and the initial transformations
 * like the IMU_T_depth, etc. matrices.
 * It calculates often used conversion between the depth and color frame,
 * or transforms a pose into the OpenGL world, etc..
 */

#include <tango_client_api.h>
#include <tango-gl/util.h>
#include <tango-gl/conversions.h>

#include "tango-scene-reconstructor/util.h"

#ifndef TANGOSCENERECONSTRUCTOR_CONVERSION_H
#define TANGOSCENERECONSTRUCTOR_CONVERSION_H

namespace tango_scene_reconstructor {

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

} // namespace tango_scene_reconstructor

#endif //TANGOSCENERECONSTRUCTOR_CONVERSION_H
