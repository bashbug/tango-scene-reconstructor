/*
 * Class writes rgb point cloud either to an file or
 * sends it via TCP socket.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H

#include <string>
#include <vector>
#include <map>
#include <tango-gl/util.h>
#include <opencv2/opencv.hpp>

#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  class PointCloudData {
    public:
      PointCloudData(int id);
      ~PointCloudData();
      void MapXYZWithRGB(glm::mat4 &ss_T_color,
                         glm::mat4 &ss_T_depth,
                         glm::mat4 &color_t1_T_depth_t0,
                         glm::mat4 &start_service_T_color_t1,
                         const std::vector <float> &render_point_cloud_buffer,
                         const std::vector<uint8_t> &image_buffer);
      /*void MapXYZWithRGB(TangoPoseData &ss_T_color,
                         glm::mat4 &color_t1_T_depth_t0,
                         glm::mat4 &start_service_T_color_t1,
                         const std::vector <float> &render_point_cloud_buffer,
                         const std::vector <uint8_t> &rgb_map_buffer,
                         const std::vector <uint32_t> &rgb_pcd_buffer);*/
      glm::mat4 GetColor_T_DepthPose() {return color_T_depth_;}
      glm::mat4 GetOpenGl_T_DepthPose() {return opengl_T_depth_;}
      void SetPCDData(const std::vector<float> pcd);
      void SetPCDWithRGBData(const std::vector<float> pcd);
      void SetTranslation(const std::vector<float> translation);
      void SetRotation(const std::vector<float> rotation);
      void SetTranslation(const glm::vec3 translation);
      void SetRotation(const glm::quat rotation);
      void SetPose(const glm::mat4 pose);
      void SetCameraIntrinsics(TangoCameraIntrinsics intrinsics);
      std::vector<float> GetPCDData();
      std::vector<float> GetPCDWithRGBData();
      glm::vec3 GetTranslation();
      glm::quat GetRotation();
      std::vector<float> GetXYZValues_T_Color();
      std::vector<float> GetXYZValues();
      std::vector<float> GetTransformedXYZValues();
      std::vector<uint8_t> GetRGBValues();
      std::vector<float> GetRGBPCDFileValues();
      int GetId();
      glm::mat4 GetPose() {return ss_T_color_;}
      glm::mat4 GetDepthPose() { return  ss_T_depth_;}
      std::vector<float> GetRangeImage();

  private:
      std::vector <float> pcd_with_rgb_data_;
      std::vector<float> xyz_T_color_buffer_;
      //std::vector<cv::Point3f> cv_vertrices_;
      std::vector<uint8_t> rgb_values_;
      glm::vec3 translation_;
      glm::quat rotation_;
      glm::mat4 color_T_depth_;
      glm::mat4 opengl_T_depth_;
      glm::mat4 ss_T_color_;
      glm::mat4 ss_T_depth_;
      std::vector<float>rgb_values_pcd_file_;
      TangoCameraIntrinsics rgb_camera_intrinsics_;
      std::vector<float> range_image_;
      int id_;
      std::vector<float> xyz_buffer_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
