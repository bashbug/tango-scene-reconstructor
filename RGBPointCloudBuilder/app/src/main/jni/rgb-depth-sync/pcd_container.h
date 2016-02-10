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
    private:
      std::shared_ptr<std::mutex> pcd_mtx_;
      std::shared_ptr<std::condition_variable> consume_pcd_;
      std::vector<PCD*> pcd_container_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H