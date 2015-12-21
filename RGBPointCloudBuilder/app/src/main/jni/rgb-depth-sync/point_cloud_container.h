/*
 * Container of all registered rgb point clouds.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H

#include <vector>

#include "tango-gl/util.h"
#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/point.h"

namespace rgb_depth_sync {

  class PointCloudContainer {
  public:
    explicit PointCloudContainer();
    ~PointCloudContainer();
    void UpdateMergedPointCloud();
    const std::vector<float> GetMergedPointCloud() const;
    void SavePointCloud(TangoPoseData* ss_T_device_cts,
                        TangoPoseData* ss_T_device_dts,
                        const std::vector <float> &render_point_cloud_buffer,
                        const std::vector <uint8_t> &rgb_map_buffer);
    void SetCameraIntrinsics(TangoCameraIntrinsics intrinsics);
    std::vector<float> GetVertices();
    std::vector<uint8_t> GetRGBValues();
    const std::vector<float> GetPreviousPointCloud() const;
    const std::vector<float> GetCurrentPointCloud() const;
  private:
    void Set();
    std::vector<float> merged_pcd_;
    std::vector<Point> pcd_container_;
    glm::mat4 device_T_color_;
    glm::mat4 device_T_depth_;
    glm::mat4 color_T_device_;
    TangoCameraIntrinsics rgb_camera_intrinsics_;
    std::vector<float> vertices_;
    std::vector<uint8_t> rgb_values_;
    std::vector<float> previous_point_cloud_;
    std::vector<float> current_point_cloud_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H