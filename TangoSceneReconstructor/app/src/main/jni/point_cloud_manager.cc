#include "tango-scene-reconstructor/point_cloud_manager.h"

namespace tango_scene_reconstructor {

  PointCloudManager::PointCloudManager(std::shared_ptr<std::mutex> xyz_mtx,
                                       std::shared_ptr<std::condition_variable> consume_xyz) {
    xyz_mtx_ = xyz_mtx;
    consume_xyz_ = consume_xyz;
    write_pcd_data_ = false;

    range_ = 2.5f;

    is_running_ = false;

    tango_mesh_reconstructor_ = new tango_scene_reconstructor::TangoMeshReconstructor(0.03f, 0.25f, 2.0f);

    point_cloud_reconstructor_ = new tango_scene_reconstructor::PointCloudReconstructor();
    point_cloud_reconstructor_ftfsm_filtered_= new tango_scene_reconstructor::PointCloudReconstructor();
    point_cloud_reconstructor_mfsm_filtered_ = new tango_scene_reconstructor::PointCloudReconstructor();
    point_cloud_reconstructor_ftfsm_downsampled_ = new tango_scene_reconstructor::PointCloudReconstructor();
    point_cloud_reconstructor_mfsm_downsampled_ = new tango_scene_reconstructor::PointCloudReconstructor();
  }

  PointCloudManager::~PointCloudManager(){
    point_cloud_reconstructor_ = nullptr;
  }

  void PointCloudManager::SetManagers(TangoSupportPointCloudManager* xyz_manager,
                                TangoSupportImageBufferManager* yuv_manager) {
      xyz_manager_ = xyz_manager;
      yuv_manager_ = yuv_manager;
    }

  void PointCloudManager::SetRangeValue(float range) {
    range_ = range;
  }

  bool PointCloudManager::IsRunning() {
    return is_running_;
  }

  void PointCloudManager::Stop() {
    write_pcd_data_ = false;
  }

  void PointCloudManager::Start() {
    write_pcd_data_ = true;
  }

  void PointCloudManager::OnPCDAvailable(){
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

            //tango_mesh_reconstructor_->Update(xyz_buffer_, yuv_buffer_);

            PointCloud* pcd = new tango_scene_reconstructor::PointCloud(range_);
            pcd->SetXYZ(xyz_buffer_);
            pcd->SetYUV(yuv_buffer_);
            pcd->Update();
            pcd->RemoveOutliers(5.0f);
            AddPCD(pcd);
          }
          is_running_ = false;
        }
      }
    }
  }

  void PointCloudManager::AddPCD(PointCloud *point_cloud) {
    point_cloud_container_.push_back(point_cloud);
    point_cloud_reconstructor_->AddPointCloud(point_cloud);
  }

  PointCloud* PointCloudManager::GetLatestPCD() {
    if (point_cloud_container_.empty()) {
      return nullptr;
    } else {
      return point_cloud_container_.back();
    }
  }

  int PointCloudManager::GetPCDContainerLastIndex() {
    return point_cloud_container_.size()-1;
  }

  std::vector<float> PointCloudManager::GetXYZValues(glm::mat4 curr_pose) {
    return point_cloud_reconstructor_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PointCloudManager::GetRGBValues() {
    return point_cloud_reconstructor_->GetRGBValues();
  }

  glm::mat4 PointCloudManager::GetCentroidMatrix() {
    return point_cloud_reconstructor_->GetCentroidMatrix();
  }

  void PointCloudManager::OptimizeMesh() {
    for (int i = 0; i < point_cloud_container_.size(); i++) {
      point_cloud_reconstructor_ftfsm_downsampled_->AddPointCloudOptWithSM(point_cloud_container_[i]);
      point_cloud_reconstructor_mfsm_downsampled_->AddPointCloudOptWithMSM(point_cloud_container_[i]);
      point_cloud_reconstructor_ftfsm_filtered_->AddPointCloudOptWithSM(point_cloud_container_[i]);
      point_cloud_reconstructor_mfsm_filtered_->AddPointCloudOptWithMSM(point_cloud_container_[i]);
    }

    point_cloud_reconstructor_ftfsm_filtered_->FilterPointCloud();
    point_cloud_reconstructor_mfsm_filtered_->FilterPointCloud();

    point_cloud_reconstructor_ftfsm_downsampled_->DownsamplePointCloud();
    point_cloud_reconstructor_mfsm_downsampled_->DownsamplePointCloud();
  }

  std::vector<float> PointCloudManager::GetXYZValuesOptWithSM(glm::mat4 curr_pose) {
    return point_cloud_reconstructor_ftfsm_downsampled_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PointCloudManager::GetRGBOptWithSMValues() {
    return point_cloud_reconstructor_ftfsm_downsampled_->GetRGBValues();
  }

  std::vector<float> PointCloudManager::GetXYZValuesOptWithMSM(glm::mat4 curr_pose) {
    return point_cloud_reconstructor_mfsm_downsampled_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PointCloudManager::GetRGBOptWithMSMValues() {
    return point_cloud_reconstructor_mfsm_downsampled_->GetRGBValues();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManager::GetFTFSMMeshPCDFile() {
    return point_cloud_reconstructor_ftfsm_filtered_->GetPCDFile();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManager::GetMFSMMeshPCDFile() {
    return point_cloud_reconstructor_mfsm_filtered_->GetPCDFile();
  }

  void PointCloudManager::ResetPCD() {
    while(!point_cloud_reconstructor_->Reset() || !point_cloud_reconstructor_ftfsm_downsampled_->Reset() || !point_cloud_reconstructor_mfsm_downsampled_->Reset()) {
      LOGE("mesh is still running");
    }
    point_cloud_container_.clear();
    LOGE("MESH abd CONATINER are reseted");
  }
} // namespace rgb_depth_syn