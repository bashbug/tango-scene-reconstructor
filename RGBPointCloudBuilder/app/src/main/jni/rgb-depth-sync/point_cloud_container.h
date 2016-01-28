/*
 * Container of all registered rgb point clouds.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H

#include <vector>
#include <map>
#include <utility>

#include <tango-gl/util.h>

#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/point_cloud_data.h"

namespace rgb_depth_sync {

  class PointCloudContainer {
  public:
    PointCloudContainer();
    ~PointCloudContainer();
    void SetPointCloudData(PointCloudData* pcd);
    PointCloudData* GetLatestPCD();
    std::vector<float> GetAllXYZValues() {return merged_point_cloud_xyz_values_;}
    std::vector<uint8_t> GetAllRGBValues() {return merged_point_cloud_rgb_values_;}
    std::vector<PointCloudData*> GetPointCloudContainer() {return pcd_container_;}
  private:
    std::vector<PointCloudData*> pcd_container_;
    std::vector<float> merged_point_cloud_xyz_values_;
    std::vector<uint8_t> merged_point_cloud_rgb_values_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H