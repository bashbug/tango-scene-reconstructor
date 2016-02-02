#ifndef RGBPOINTCLOUDBUILDER_PCD_WORKER_H
#define RGBPOINTCLOUDBUILDER_PCD_WORKER_H

#include <mutex>
#include <condition_variable>
#include <thread>
#include <future> //async
#include <chrono> //time

#include <tango_client_api.h>
#include <tango-gl/util.h>

#include <opencv2/opencv.hpp>

#include "rgb-depth-sync/pcd.h"
#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  class PCDWorker {
    public:
      PCDWorker(PCDContainer* pcd_container);
      ~PCDWorker();
      void SetXYZBuffer(const TangoXYZij* xyz_buffer);
      void SetRGBBuffer(const TangoImageBuffer* yuv_buffer);
      void OnPCDAvailable();
    private:
      std::vector<float> xyz_;
      double xyz_timestamp_;
      std::vector<uint8_t> rgb_;
      double rgb_timestamp_;
      std::mutex data_mtx_;
      std::condition_variable consume_data_;
      cv::Mat yuv_frame_;
      cv::Mat rgb_frame_;
      bool xyz_set_;
      bool rgb_set_;
      PCDContainer* pcd_container_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_PCD_WORKER_H
