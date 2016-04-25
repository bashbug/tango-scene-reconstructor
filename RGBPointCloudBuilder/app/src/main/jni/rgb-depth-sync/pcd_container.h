/*
 * PCDContainer class holds all registered rgb point clouds while a scanning process.
 * It will be reset if the scanning process (re)starts.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H

#include <memory> // shared pointer
#include <mutex>
#include <condition_variable>
#include <vector>
#include <tango_client_api.h>
#include <tango-gl/util.h>

#include "rgb-depth-sync/pcd.h"
#include "rgb-depth-sync/mesh.h"

namespace rgb_depth_sync {
  class PCDContainer {
    public:
      PCDContainer();
      ~PCDContainer();
      void AddPCD(PCD *pcd);
      PCD* GetLatestPCD();
      int GetPCDContainerLastIndex();
      std::vector<float> GetXYZValues(glm::mat4 curr_pose);
      std::vector<uint8_t> GetRGBValues();
      std::vector<float> GetXYZValuesOptWithSM(glm::mat4 curr_pose);
      std::vector<float> GetXYZValuesOptWithMSM(glm::mat4 curr_pose);
      std::vector<uint8_t> GetRGBOptWithSMValues();
      std::vector<uint8_t> GetRGBOptWithMSMValues();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetFTFSMMeshPCDFile();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetMFSMMeshPCDFile();
      void OptimizeMesh();
      void ResetPCD();
      std::vector<PCD*> pcd_container_;
    private:
      Mesh* mesh_;
      Mesh* mesh_sm_filtered_;
      Mesh* mesh_msm_filtered_;
      Mesh* mesh_sm_downsampled_;
      Mesh* mesh_msm_downsampled_;
  };
} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H