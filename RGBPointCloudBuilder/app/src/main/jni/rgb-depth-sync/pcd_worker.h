#ifndef RGBPOINTCLOUDBUILDER_PCD_WORKER_H
#define RGBPOINTCLOUDBUILDER_PCD_WORKER_H

#include <tango_support_api.h>

#include <mutex>
#include <thread>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp> // ORB and BFMatcher
#include <opencv2/highgui.hpp>
//#include <opencv2/xfeatures2d.hpp>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "rgb-depth-sync/pcd.h"
#include "rgb-depth-sync/pcd_container.h"
#include "rgb-depth-sync/pcd_outlier_removal.h"

namespace rgb_depth_sync {

  class PCDWorker {
    public:
      PCDWorker(PCDContainer* pcd_container, TangoSupportPointCloudManager* xyz_manager, TangoSupportImageBufferManager* yuv_manager);
      ~PCDWorker();
      void SetXYZBuffer(const TangoXYZij* xyz_buffer);
      void SetRGBBuffer(const TangoImageBuffer* yuv_buffer);
      void OnPCDAvailable();
      void StopPCDWorker();
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
      int pcd_count_;
      int img_count_;
      bool write_pcd_data_;
      cv::Ptr<cv::ORB> orb_;
      PCDOutlierRemoval* pcd_remove_outlier_;
      TangoPoseData* curr_pose_, prev_pose_;
      TangoSupportPointCloudManager* xyz_manager_;
      TangoSupportImageBufferManager* yuv_manager_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_PCD_WORKER_H
