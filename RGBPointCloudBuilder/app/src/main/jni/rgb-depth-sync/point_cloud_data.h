/*
 * Class writes rgb point cloud either to an file or
 * sends it via TCP socket.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H

#include "tango-gl/util.h"
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  class PointCloudData {
    public:
      explicit PointCloudData();
      ~PointCloudData();
      void SaveOrderedPointCloudToFile(const std::vector<float> point_cloud,
                                       const glm::vec3 translation,
                                       const glm::quat rotation,
                                       int width, int height,
                                       const double timestamp);
      void setOrdered(int width, int height);
      void setUnordered();
      void saveToFile();
      void saveToSocket(std::string addr, int port);
      std::vector<float> getPCDData();
      std::vector<float> GetRGBDData();
      std::vector<uint8_t> GetRGBData();
      void setPCDData(const std::vector<float> point_cloud,
                      const glm::vec3 translation,
                      const glm::quat rotation,
                      const double timestamp);
      void SetRGBDData(glm::mat4 &color_t1_T_depth_t0,
                       glm::mat4 &start_service_T_color_t1,
                       const std::vector <float> &render_point_cloud_buffer,
                       const std::vector <uint8_t> &rgb_map_buffer,
                       const std::vector <uint32_t> &rgb_pcd_buffer);
      void SetCameraIntrinsics(TangoCameraIntrinsics intrinsics);
      std::string timestampToString(double value);
      std::string getHeader();
      std::vector<float> GetVertices();
      std::vector<uint8_t> GetRGBValues();
      void setHeader(int width, int height);

    private:
      std::string header_;
      std::string timestamp_;
      std::vector <float> pcd_;
      std::vector<float> vertices_;
      std::vector<uint8_t> rgb_values_;
      glm::vec3 translation_;
      glm::quat rotation_;
      size_t pcd_file_counter_ = 1;
      std::vector<float> transformed_unordered_point_cloud_to_image_frame_;
      std::vector<uint8_t> rgb_data_;
      TangoCameraIntrinsics rgb_camera_intrinsics_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
