/*
 * Container of all registered rgb point clouds.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H

#include <memory>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <tango_client_api.h>
#include <tango-gl/util.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "rgb-depth-sync/pcd.h"
#include "rgb-depth-sync/conversion.h"

namespace rgb_depth_sync {

  struct Point {
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };

  class PCDContainer {
    public:
      PCDContainer(std::shared_ptr<std::mutex> pcd_mtx, std::shared_ptr<std::condition_variable> consume_pcd);
      ~PCDContainer();
      void AddPCD(PCD *pcd);
      PCD* GetLatestPCD();
      // Should only be used from waiting thread
      int GetPCDContainerLastIndex();
      std::vector<PCD*>* GetPCDContainer();
      void ResetPCD();
      void GetXYZRGBValues(std::vector<float>* xyz, std::vector<uint8_t>* rgb, glm::mat4* ss_T_device) ;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetMergedPCD();
    private:
      std::shared_ptr<std::mutex> pcd_mtx_;
      std::shared_ptr<std::condition_variable> consume_pcd_;
      std::vector<PCD*> pcd_container_;
      float resolution_;
      std::vector<float> xyz_;
      std::vector<uint8_t> rgb_;
      std::map<int, std::map<int, std::map <int, Point> > > xyz_rgb_map_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_pcd_;
      glm::mat4 ss_T_device_;
      Conversion* conversion_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H