#include "rgb-depth-sync/pcd_worker.h"

namespace rgb_depth_sync {

  PCDWorker::PCDWorker(std::shared_ptr<std::mutex> xyz_mtx,
                       std::shared_ptr<std::condition_variable> consume_xyz,
                       PCDContainer* pcd_container) {
    xyz_mtx_ = xyz_mtx;
    consume_xyz_ = consume_xyz;
    write_pcd_data_ = false;

    pcd_container_ = pcd_container;
    range_ = 5.0f;

    is_running_ = false;
  }

  PCDWorker::~PCDWorker() {

  }

  void PCDWorker::SetManagers(TangoSupportPointCloudManager* xyz_manager,
                              TangoSupportImageBufferManager* yuv_manager) {
    xyz_manager_ = xyz_manager;
    yuv_manager_ = yuv_manager;
  }

  void PCDWorker::SetRangeValue(float range) {
    range_ = range;
  }

  bool PCDWorker::IsRunning() {
    return is_running_;
  }

  void PCDWorker::Stop() {
    write_pcd_data_ = false;
  }

  void PCDWorker::Start() {
    write_pcd_data_ = true;
  }

  void PCDWorker::OnPCDAvailable(){
    TangoXYZij* xyz = new TangoXYZij();
    TangoImageBuffer* yuv = new TangoImageBuffer();
    bool new_xyz_data = false;
    bool new_yuv_data = false;

    std::unique_lock<std::mutex> lock(*xyz_mtx_);

    while(true) {
      consume_xyz_->wait(lock);
      if (write_pcd_data_) {
        new_xyz_data = false;
        TangoSupport_getLatestPointCloudAndNewDataFlag(xyz_manager_, &xyz_buffer_, &new_xyz_data);

        if (new_xyz_data) {
          is_running_ = true;
          int ret = TangoSupport_getLatestImageBuffer(yuv_manager_, &yuv_buffer_);
          if (ret == TANGO_SUCCESS) {

            PCD* pcd = new rgb_depth_sync::PCD();
            pcd->SetXYZ(xyz_buffer_);
            pcd->SetYUV(yuv_buffer_);
            pcd->Update();
            pcd->RemoveOutliers(range_);
            pcd_container_->AddPCD(pcd);

          }
          is_running_ = false;
        }
      }
    }
  }
}