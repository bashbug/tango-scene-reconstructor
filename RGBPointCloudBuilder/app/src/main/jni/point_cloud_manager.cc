#include "tango-scene-reconstructor/point_cloud_manager.h"

namespace tango_scene_reconstructor {

  PointCloudManager::PointCloudManager(std::shared_ptr<std::mutex> xyz_mtx,
                                       std::shared_ptr<std::condition_variable> consume_xyz) {
    xyz_mtx_ = xyz_mtx;
    consume_xyz_ = consume_xyz;
    write_pcd_data_ = false;

    range_ = 5.0f;

    is_running_ = false;

    mesh_ = new tango_scene_reconstructor::PointCloudReconstructor();
    mesh_sm_filtered_ = new tango_scene_reconstructor::PointCloudReconstructor();
    mesh_msm_filtered_ = new tango_scene_reconstructor::PointCloudReconstructor();
    mesh_sm_downsampled_ = new tango_scene_reconstructor::PointCloudReconstructor();
    mesh_msm_downsampled_ = new tango_scene_reconstructor::PointCloudReconstructor();
  }

  PointCloudManager::~PointCloudManager(){
    mesh_ = nullptr;
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

            PointCloud* pcd = new tango_scene_reconstructor::PointCloud();
            pcd->SetXYZ(xyz_buffer_);
            pcd->SetYUV(yuv_buffer_);
            pcd->Update();
            pcd->RemoveOutliers(range_);
            AddPCD(pcd);
          }
          is_running_ = false;
        }
      }
    }
  }

  void PointCloudManager::AddPCD(PointCloud *point_cloud) {
    point_cloud_container_.push_back(point_cloud);
    mesh_->AddPointCloud(point_cloud);
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
    LOGE("Render PCD reconstruction");
    return mesh_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PointCloudManager::GetRGBValues() {
    return mesh_->GetRGBValues();
  }

  glm::mat4 PointCloudManager::GetCentroidMatrix() {
    return mesh_->GetCentroidMatrix();
  }

  void PointCloudManager::OptimizeMesh() {
    for (int i = 0; i < point_cloud_container_.size(); i++) {
      mesh_sm_downsampled_->AddPointCloudOptWithSM(point_cloud_container_[i]);
      mesh_msm_downsampled_->AddPointCloudOptWithMSM(point_cloud_container_[i]);
      mesh_sm_filtered_->AddPointCloudOptWithSM(point_cloud_container_[i]);
      mesh_msm_filtered_->AddPointCloudOptWithMSM(point_cloud_container_[i]);
    }

    mesh_sm_filtered_->FilterMesh();
    mesh_msm_filtered_->FilterMesh();

    mesh_sm_downsampled_->DownsampleMesh();
    mesh_msm_downsampled_->DownsampleMesh();
  }

  std::vector<float> PointCloudManager::GetXYZValuesOptWithSM(glm::mat4 curr_pose) {
    return mesh_sm_downsampled_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PointCloudManager::GetRGBOptWithSMValues() {
    return mesh_sm_downsampled_->GetRGBValues();
  }

  std::vector<float> PointCloudManager::GetXYZValuesOptWithMSM(glm::mat4 curr_pose) {
    return mesh_msm_downsampled_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PointCloudManager::GetRGBOptWithMSMValues() {
    return mesh_msm_downsampled_->GetRGBValues();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManager::GetFTFSMMeshPCDFile() {
    return mesh_sm_filtered_->GetPCDFile();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudManager::GetMFSMMeshPCDFile() {
    return mesh_msm_filtered_->GetPCDFile();
  }

  void PointCloudManager::ResetPCD() {
    while(!mesh_->Reset() || !mesh_msm_downsampled_->Reset() || !mesh_msm_downsampled_->Reset()) {
      LOGE("mesh is still running");
    }
    point_cloud_container_.clear();
    LOGE("MESH abd CONATINER are reseted");
  }
} // namespace rgb_depth_syn