#ifndef RGBPOINTCLOUDBUILDER_PCD_WORKER_H
#define RGBPOINTCLOUDBUILDER_PCD_WORKER_H

#include <mutex>
#include <thread>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp> // ORB and BFMatcher
//#include <opencv2/xfeatures2d.hpp>
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
      void Yuv2Rgb(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* gray);
      std::vector<float> xyz_;
      double xyz_timestamp_;
      std::vector<uint8_t> rgb_;
      std::vector<uint8_t> gray_;
      double rgb_timestamp_;
      std::mutex data_mtx_;
      std::condition_variable consume_data_;
      std::vector<uint8_t> yuv_;
      uint32_t yuv_height_;
      uint32_t yuv_width_;
      uint32_t uv_offset_;
      uint32_t yuv_size_;
      cv::Mat yuv_frame_;
      cv::Mat rgb_frame_;
      cv::Mat gray_frame_;
      bool xyz_set_;
      bool rgb_set_;
      int rgb_size_;
      PCDContainer* pcd_container_;
      //cv::Ptr<cv::SIFT> sift_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_PCD_WORKER_H
