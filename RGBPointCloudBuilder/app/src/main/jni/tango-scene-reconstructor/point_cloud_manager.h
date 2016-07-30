/*
 * PCDContainer class holds all registered rgb point clouds while a scanning process.
 * It will be reset if the scanning process (re)starts.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H

#include <memory> // shared pointer
#include <mutex>
#include <thread>
#include <condition_variable>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <tango_support_api.h>
#include <tango_client_api.h>
#include <tango-gl/util.h>

#include "tango-scene-reconstructor/point_cloud.h"
#include "tango-scene-reconstructor/point_cloud_reconstructor.h"

namespace tango_scene_reconstructor {
  class PointCloudManager {
    public:
      PointCloudManager(std::shared_ptr<std::mutex> xyz_mtx,
                   std::shared_ptr<std::condition_variable> consume_xyz);
      ~PointCloudManager();
      void AddPCD(PointCloud *point_cloud);
      PointCloud* GetLatestPCD();
      int GetPCDContainerLastIndex();
      std::vector<float> GetXYZValues(glm::mat4 curr_pose);
      std::vector<uint8_t> GetRGBValues();
      std::vector<float> GetXYZValuesOptWithSM(glm::mat4 curr_pose);
      std::vector<float> GetXYZValuesOptWithMSM(glm::mat4 curr_pose);
      std::vector<uint8_t> GetRGBOptWithSMValues();
      std::vector<uint8_t> GetRGBOptWithMSMValues();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetFTFSMMeshPCDFile();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetMFSMMeshPCDFile();
      glm::mat4 GetCentroidMatrix();
      void OptimizeMesh();
      void ResetPCD();
      std::vector<PointCloud*> point_cloud_container_;
      void OnPCDAvailable();
      void SetManagers(TangoSupportPointCloudManager* xyz_manager,
                       TangoSupportImageBufferManager* yuv_manager);
      void Stop();
      void Start();
      bool IsRunning();
      void SetRangeValue(float range);
    private:
      PointCloudReconstructor* mesh_;
      PointCloudReconstructor* mesh_sm_filtered_;
      PointCloudReconstructor* mesh_msm_filtered_;
      PointCloudReconstructor* mesh_sm_downsampled_;
      PointCloudReconstructor* mesh_msm_downsampled_;
      std::shared_ptr<std::mutex> xyz_mtx_;
      std::shared_ptr<std::condition_variable> consume_xyz_;
      TangoSupportPointCloudManager* xyz_manager_;
      TangoSupportImageBufferManager* yuv_manager_;
      bool write_pcd_data_;
      std::vector<float> xyz_;
      std::vector<uint8_t> rgb_;
      double xyz_timestamp_;
      double rgb_timestamp_;
      uint32_t yuv_size_;
      uint32_t rgb_size_;
      cv::Mat yuv_frame_;
      cv::Mat rgb_frame_;
      cv::Mat gray_frame_;
      cv::Mat gray_frame_320x180_;
      TangoXYZij* xyz_buffer_;
      TangoImageBuffer* yuv_buffer_;
      bool new_xyz_data;
      bool new_yuv_data;
      cv::Ptr<cv::ORB> orb_;
      bool is_running_;
      float range_;
  };
} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_CONTAINER_H