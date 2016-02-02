#include "rgb-depth-sync/pcd_worker.h"

namespace rgb_depth_sync {

  PCDWorker::PCDWorker(PCDContainer* pcd_container) {
    pcd_container_ = pcd_container;
    xyz_set_ = false;
    rgb_set_ = false;
    yuv_frame_.create(720 * 3 / 2, 1280, CV_8UC1);
    rgb_frame_.create(720, 1280, CV_8UC3);
  }

  PCDWorker::~PCDWorker() {

  }

  void PCDWorker::SetXYZBuffer(const TangoXYZij* xyz_buffer){
    std::unique_lock<std::mutex> lock(data_mtx_);
    if (xyz_set_ == true)
      return;
    xyz_timestamp_ = xyz_buffer->timestamp;
    size_t point_cloud_size = xyz_buffer->xyz_count * 3;
    std::vector<float> xyz_tmp;
    xyz_.clear();
    xyz_.resize(point_cloud_size);
    std::copy(xyz_buffer->xyz[0], xyz_buffer->xyz[0] + point_cloud_size, xyz_.begin());
    xyz_set_ = true;
    consume_data_.notify_one();
   }

  void PCDWorker::SetRGBBuffer(const TangoImageBuffer* yuv_buffer){
    std::unique_lock<std::mutex> lock(data_mtx_);
    if (rgb_set_ == true)
      return;
    cv::Mat yuv_frame, rgb_frame;
    memcpy(yuv_frame_.data, yuv_buffer->data, 1280 * 720 * 3 / 2);
    rgb_timestamp_ = yuv_buffer->timestamp;
    rgb_set_ = true;
    consume_data_.notify_one();
  }

  void PCDWorker::OnPCDAvailable(){
    std::unique_lock<std::mutex> lock(data_mtx_);
    while(true) {
      consume_data_.wait(lock);
      if(xyz_set_ && rgb_set_) {
        PCD *pcd = new rgb_depth_sync::PCD();
        cv::cvtColor(yuv_frame_, rgb_frame_, CV_YUV2RGB_NV21);
        rgb_.clear();
        rgb_.resize(1280 * 720 * 3);
        memcpy(&rgb_[0], rgb_frame_.data, 1280 * 720 * 3);
        pcd->MapXYZWithRGB(xyz_, rgb_, xyz_timestamp_, rgb_timestamp_);
        pcd_container_->AddPCD(pcd);
        rgb_set_ = false;
        xyz_set_ = false;
      }
    }
  }
}