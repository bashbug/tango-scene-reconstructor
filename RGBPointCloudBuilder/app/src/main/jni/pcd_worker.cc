#include "rgb-depth-sync/pcd_worker.h"

namespace rgb_depth_sync {

  PCDWorker::PCDWorker(std::shared_ptr<std::mutex> xyz_mtx,
                       std::shared_ptr<std::condition_variable> consume_xyz,
                       PCDContainer* pcd_container,
                       TangoSupportPointCloudManager* xyz_manager,
                       TangoSupportImageBufferManager* yuv_manager) {
    xyz_mtx_ = xyz_mtx;
    consume_xyz_ = consume_xyz;
    xyz_manager_ = xyz_manager;
    yuv_manager_ = yuv_manager;
    write_pcd_data_ = true;
    yuv_frame_.create(720*3/2, 1280, CV_8UC1);
    rgb_frame_.create(720, 1280, CV_8UC3);
    rgb_size_ = 720*1280*3;
    yuv_size_ = 720*3/2*1280;
    pcd_container_ = pcd_container;
  }

  PCDWorker::~PCDWorker() {

  }

  void PCDWorker::StopPCDWorker() {
    write_pcd_data_ = false;
  }

  void PCDWorker::OnPCDAvailable(){
    TangoXYZij* xyz = new TangoXYZij();
    TangoImageBuffer* yuv = new TangoImageBuffer();
    bool new_xyz_data = false;
    bool new_yuv_data = false;

    std::unique_lock<std::mutex> lock(*xyz_mtx_);

    while(write_pcd_data_) {
      consume_xyz_->wait(lock);
      new_xyz_data = false;
      TangoSupport_getLatestPointCloudAndNewDataFlag(xyz_manager_, &xyz_buffer_, &new_xyz_data);

      if (new_xyz_data) {
        int ret = TangoSupport_getLatestImageBuffer(yuv_manager_, &yuv_buffer_);
        if (ret == TANGO_SUCCESS) {
          /// copy xyz buffer
          xyz_.clear();
          xyz_.resize(xyz_buffer_->xyz_count * 3);
          std::copy(xyz_buffer_->xyz[0], xyz_buffer_->xyz[0] + xyz_buffer_->xyz_count * 3, xyz_.begin());
          // copy yuv buffer
          memcpy(yuv_frame_.data, yuv_buffer_->data, yuv_size_);
          cv::cvtColor(yuv_frame_, rgb_frame_, CV_YUV2RGB_NV21);
          rgb_.clear();
          rgb_.resize(rgb_size_);
          memcpy(&rgb_[0], rgb_frame_.data, rgb_size_);
          PCD* pcd = new rgb_depth_sync::PCD();
          //pcd->SetTangoXYZij(xyz_buffer_);
          // sync xyz and rgb
          PCDOutlierRemoval pcd_outlier_removal;
          std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > xyz_without_outliers = pcd_outlier_removal.Compute(xyz_, 0.25f, 1.0f, 10);
          if (xyz_without_outliers.size() > 0) {
            pcd->MapXYZWithRGB(xyz_without_outliers, rgb_, xyz_buffer_->timestamp, yuv_buffer_->timestamp);
          }
          if (pcd->GetPCD().size() > 0) {
            pcd_container_->AddPCD(pcd);
          }
        }
      }
    }
  }
}