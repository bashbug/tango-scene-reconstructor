/*
 * Class writes rgb point cloud either to an file or
 * sends it via TCP socket.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H

#include <vector>
#include <tango-gl/util.h>
#include <tango_client_api.h>
#include <rgb-depth-sync/pose_data.h>
#include <rgb-depth-sync/util.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace rgb_depth_sync {

  class PCD {
  public:
    PCD();
    ~PCD();
    void MapXYZWithRGB(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >& xyz,
                       const std::vector<uint8_t>& rgb,
                       double xyz_timestamp,
                       double rgb_timestamp);
    std::vector<float> GetXYZValues();
    std::vector<uint8_t> GetRGBValues();
    glm::mat4 GetPose();
    glm::vec3 GetTranslation();
    glm::quat GetRotation();
    void SetTranslation(const glm::vec3& translation);
    void SetRotation(const glm::quat rotation);
    std::vector<float> GetXYZValuesTSS();
    std::vector<float> GetPCD();
  private:
    std::vector<float> xyz_values_color_camera_;
    std::vector<float> xyz_values_ss_;
    std::vector<uint8_t> rgb_values_;
    std::vector<float> pcd_;
    double xyz_timestamp_;
    double rgb_timestamp_;
    glm::mat4 pose_;
    glm::quat rotation_;
    glm::vec3 translation_;
    Eigen::Matrix4f transformation_matrix_;
  };
} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
