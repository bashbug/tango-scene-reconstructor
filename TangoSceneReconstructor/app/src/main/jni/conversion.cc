#include "tango-scene-reconstructor/conversion.h"

namespace tango_scene_reconstructor {

  Conversion* Conversion::GetInstance() {
    static Conversion* instance = new tango_scene_reconstructor::Conversion();
    static bool set = false;
    if (!set) {
      instance->SetMatrices();
      set = true;
    }
    return instance;
  }

  void Conversion::SetMatrices() {
    TangoPoseData pose_imu_T_device;
    TangoPoseData pose_imu_T_color;
    TangoPoseData pose_imu_T_depth;
    TangoCoordinateFramePair frame_pair;

    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_device);

    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
    TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_color);

    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
    TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_depth);

    glm::mat4 imu_T_device = util::GetMatrixFromPose(&pose_imu_T_device);
    glm::mat4 imu_T_color = util::GetMatrixFromPose(&pose_imu_T_color);
    glm::mat4 imu_T_depth = util::GetMatrixFromPose(&pose_imu_T_depth);

    device_T_color_ = glm::inverse(imu_T_device) * imu_T_color;
    device_T_depth_ = glm::inverse(imu_T_device) * imu_T_depth;
    color_T_device_ = glm::inverse(device_T_color_);
  }

  glm::mat4 Conversion::XYZ_T_RGB(const double &rgb_timestamp, const double &xyz_timestamp) {

    TangoPoseData pose_ss_T_device_xyz_timestamp;
    TangoCoordinateFramePair xyz_frame_pair;
    xyz_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    xyz_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoService_getPoseAtTime(xyz_timestamp, xyz_frame_pair, &pose_ss_T_device_xyz_timestamp);
    if(pose_ss_T_device_xyz_timestamp.status_code != TANGO_POSE_VALID) {
      LOGE("xyz pose invalid");
    }

    TangoPoseData pose_ss_T_device_rgb_timestamp;
    TangoCoordinateFramePair rgb_frame_pair;
    rgb_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    rgb_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoService_getPoseAtTime(rgb_timestamp, rgb_frame_pair, &pose_ss_T_device_rgb_timestamp);

    if(pose_ss_T_device_rgb_timestamp.status_code != TANGO_POSE_VALID) {
      LOGE("rgb pose invalid");
    }

    glm::mat4 ss_T_device_xyz_timestamp = util::GetMatrixFromPose(&pose_ss_T_device_xyz_timestamp);
    glm::mat4 ss_T_device_rgb_timestamp = util::GetMatrixFromPose(&pose_ss_T_device_rgb_timestamp);

    return color_T_device_ * glm::inverse(ss_T_device_rgb_timestamp)
           * ss_T_device_xyz_timestamp * device_T_depth_;
  }

  glm::mat4 Conversion::SS_T_RGB(const double &rgb_timestamp) {

    TangoPoseData pose_ss_T_device_rgb_timestamp;
    TangoCoordinateFramePair rgb_frame_pair;
    rgb_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    rgb_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoService_getPoseAtTime(rgb_timestamp, rgb_frame_pair, &pose_ss_T_device_rgb_timestamp);

    return util::GetMatrixFromPose(&pose_ss_T_device_rgb_timestamp) * device_T_color_;
  }

  glm::mat4 Conversion::OpenGL_T_RGB(const glm::mat4 &ss_T_rgb) {
    return tango_gl::conversions::opengl_world_T_tango_world() * ss_T_rgb;
  }

  glm::mat4 Conversion::OpenGL_T_OpenGLCamera(const glm::mat4 &ss_T_rgb) {
    return tango_gl::conversions::opengl_world_T_tango_world() * ss_T_rgb * tango_gl::conversions::color_camera_T_opengl_camera();
  }

  glm::mat4 Conversion::OpenGL_T_Device(const glm::mat4 &ss_T_device) {
    return tango_gl::conversions::opengl_world_T_tango_world() * ss_T_device;
  }

  glm::mat4 Conversion::OpenGL_T_device_T_OpenGLCamera(const glm::mat4 &ss_T_device) {
    return tango_gl::conversions::opengl_world_T_tango_world() * ss_T_device;
  }

}
