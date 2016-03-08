#ifndef RGBPOINTCLOUDBUILDER_PCD_WORKER_H
#define RGBPOINTCLOUDBUILDER_PCD_WORKER_H

#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>         // std::chrono::seconds

#include <tango_support_api.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "rgb-depth-sync/pcd_outlier_removal.h"

#include "rgb-depth-sync/pcd_container.h"
#include "rgb-depth-sync/pcd.h"

namespace rgb_depth_sync {

  class PCDWorker {
    public:
      PCDWorker(std::shared_ptr<std::mutex> xyz_mtx,
                std::shared_ptr<std::condition_variable> consume_xyz,
                PCDContainer* pcd_container,
                TangoSupportPointCloudManager* xyz_manager,
                TangoSupportImageBufferManager* yuv_manager);
      ~PCDWorker();
      void OnPCDAvailable();
      void StopPCDWorker();
    private:
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
      PCDContainer* pcd_container_;

      TangoXYZij* xyz_buffer_;
      TangoImageBuffer* yuv_buffer_;
      bool new_xyz_data;
      bool new_yuv_data;
  };
}

#endif //RGBPOINTCLOUDBUILDER_PCD_WORKER_H
