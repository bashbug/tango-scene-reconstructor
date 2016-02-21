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
#include "rgb-depth-sync/pcd.h"

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
      std::pair<std::vector<float>, std::vector<uint8_t> > GetXYZRGBValues();
    private:
      std::shared_ptr<std::mutex> pcd_mtx_;
      std::shared_ptr<std::condition_variable> consume_pcd_;
      std::vector<PCD*> pcd_container_;
      float resolution_;
      std::vector<float> xyz_;
      std::vector<uint8_t> rgb_;
      std::map<int, std::map<int, std::map <int, Point> > > xyz_rgb_map_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H