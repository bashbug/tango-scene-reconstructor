#include "rgb-depth-sync/pcd_worker.h"

namespace rgb_depth_sync {

  PCDWorker::PCDWorker(PCDContainer* pcd_container) {
    pcd_container_ = pcd_container;
    write_pcd_data_ = true;
    xyz_set_ = false;
    rgb_set_ = false;
    yuv_frame_.create(720*3/2, 1280, CV_8UC1);
    rgb_frame_.create(720, 1280, CV_8UC3);
    rgb_size_ = 720*1280*3;
    yuv_size_ = 720*3/2*1280;
    pcd_count_ = 0;
    img_count_ = 0;
  }

  PCDWorker::~PCDWorker() {

  }

  void PCDWorker::SetXYZBuffer(const TangoXYZij* xyz_buffer){
    if(write_pcd_data_ == true) {
      std::unique_lock<std::mutex> lock(data_mtx_);
      if (xyz_set_ == true)
        return;
      xyz_timestamp_ = xyz_buffer->timestamp;
      size_t point_cloud_size = xyz_buffer->xyz_count * 3;
      xyz_.clear();
      xyz_.resize(point_cloud_size);
      std::copy(xyz_buffer->xyz[0], xyz_buffer->xyz[0] + point_cloud_size, xyz_.begin());
      xyz_set_ = true;
      pcd_count_++;
      consume_data_.notify_one();
    }
   }

  void PCDWorker::SetRGBBuffer(const TangoImageBuffer* yuv_buffer) {
    if (write_pcd_data_ == true) {
      std::unique_lock<std::mutex> lock(data_mtx_);
      if (rgb_set_ == true)
        return;

      memcpy(yuv_frame_.data, yuv_buffer->data, yuv_size_);
      rgb_timestamp_ = yuv_buffer->timestamp;
      rgb_set_ = true;
      img_count_++;
      consume_data_.notify_one();
    }
  }

  void PCDWorker::StopPCDWorker() {
    write_pcd_data_ = false;
  }

  void PCDWorker::OnPCDAvailable(){
    std::unique_lock<std::mutex> lock(data_mtx_);
    while(write_pcd_data_ == true) {
      consume_data_.wait(lock);
      if(xyz_set_ && rgb_set_) {
        PCD *pcd = new rgb_depth_sync::PCD();

        cv::cvtColor(yuv_frame_, rgb_frame_, CV_YUV2RGB_NV21);
        rgb_.clear();
        rgb_.resize(rgb_size_);
        memcpy(&rgb_[0], rgb_frame_.data, rgb_size_);

        pcd->MapXYZWithRGB(xyz_, rgb_, xyz_timestamp_, rgb_timestamp_);

        cv::Mat gray_frame;
        cv::cvtColor(rgb_frame_, gray_frame, CV_RGB2GRAY);

        pcd->SetFrame(gray_frame);

        pcd_container_->AddPCD(pcd);
        // push back the new pcd into the pcd_container
        rgb_set_ = false;
        xyz_set_ = false;
      }
      //LOGE("PCD WORKER THREAD pcd_counter: %i, img_counter: %i", pcd_count_, img_count_);
    }
  }

  void PCDWorker::Yuv2Rgb(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t* r, uint8_t* g, uint8_t* b, uint8_t* gray) {
    *r = yValue + (1.370705f * (vValue - 128.0f));
    *g = yValue - (0.698001f * (vValue - 128.0f)) - (0.337633f * (uValue - 128.0f));
    *b = yValue + (1.732446f * (uValue - 128.0f));
    *gray = 0.2989 * (*r) + 0.5870 * (*g) + 0.1140 * (*b);
  }
}