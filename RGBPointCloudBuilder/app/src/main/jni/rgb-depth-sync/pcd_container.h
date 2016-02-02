/*
 * Container of all registered rgb point clouds.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H

#include <mutex>
#include <condition_variable>
#include <vector>

#include <tango-gl/util.h>

#include "rgb-depth-sync/pcd.h"

namespace rgb_depth_sync {

  class PCDContainer {
    public:
      PCDContainer(std::mutex *pcd_mtx, std::condition_variable *consume_pcd);
      ~PCDContainer();
      void AddPCD(PCD *pcd);
      PCD* GetLatestPCD();
      int GetPCDContainerLastIndex();
    private:
      std::vector<PCD*> pcd_container_;
      std::mutex pcd_mtx_;
      std::condition_variable consume_pcd_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H