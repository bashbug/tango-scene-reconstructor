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
    write_pcd_data_ = false;
    yuv_frame_.create(720*3/2, 1280, CV_8UC1);
    rgb_frame_.create(720, 1280, CV_8UC3);
    rgb_size_ = 720*1280*3;
    yuv_size_ = 720*3/2*1280;
    pcd_container_ = pcd_container;

    orb_ = cv::ORB::create(400);
    pcd_remove_outlier_ = new rgb_depth_sync::PCDOutlierRemoval();

    is_running_ = false;
  }

  PCDWorker::~PCDWorker() {

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
            std::clock_t start = std::clock();
            std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > xyz_without_outliers = pcd_remove_outlier_->Compute(xyz_, 0.25f, 1.0f, 10);
            int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
            LOGE("Outlier filtering  ----- time %i", diff);
            if (xyz_without_outliers.size() > 0) {
              std::clock_t start_1 = std::clock();
              pcd->MapXYZWithRGB(xyz_without_outliers, rgb_, xyz_buffer_->timestamp, yuv_buffer_->timestamp);
              diff = (std::clock() - start_1) / (double)(CLOCKS_PER_SEC / 1000);
              LOGE("RGBD sync -------------- time %i", diff);
            }
            if (pcd->GetPCD().size() > 0) {
              cv::Size size(320, 180);
              cv::cvtColor(rgb_frame_, gray_frame_, CV_RGB2GRAY);
              cv::resize(gray_frame_, gray_frame_320x180_, size);
              std::vector<cv::KeyPoint> keypoints;
              cv::Mat descriptors;
              std::clock_t start_2 = std::clock();
              orb_->detectAndCompute(gray_frame_320x180_, cv::noArray(), keypoints, descriptors);
              pcd->SetKeyPointsAndDescriptors(keypoints, descriptors);
              pcd->SetFrame(gray_frame_320x180_);
              diff = (std::clock() - start_2) / (double)(CLOCKS_PER_SEC / 1000);
              LOGE("Features detection  ---- time %i", diff);
              pcd_container_->AddPCD(pcd);
            }
          }
          is_running_ = false;
        }
      }
    }
  }
}