#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  PCDContainer::PCDContainer() {
    mesh_ = new rgb_depth_sync::Mesh();
    mesh_sm_filtered_ = new rgb_depth_sync::Mesh();
    mesh_msm_filtered_ = new rgb_depth_sync::Mesh();
    mesh_sm_downsampled_ = new rgb_depth_sync::Mesh();
    mesh_msm_downsampled_ = new rgb_depth_sync::Mesh();
  }

  PCDContainer::~PCDContainer(){
    mesh_ = nullptr;
  }

  void PCDContainer::AddPCD(PCD *pcd) {
    pcd_container_.push_back(pcd);
    mesh_->AddPointCloud(pcd);
  }

  PCD* PCDContainer::GetLatestPCD() {
    if (pcd_container_.empty()) {
      return nullptr;
    } else {
      return pcd_container_.back();
    }
  }

  int PCDContainer::GetPCDContainerLastIndex() {
    return pcd_container_.size()-1;
  }

  std::vector<float> PCDContainer::GetXYZValues(glm::mat4 curr_pose) {
    return mesh_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PCDContainer::GetRGBValues() {
    return mesh_->GetRGBValues();
  }

  void PCDContainer::OptimizeMesh() {
    for (int i = 0; i < pcd_container_.size(); i++) {
      mesh_sm_downsampled_->AddPointCloudOptWithSM(pcd_container_[i]);
      mesh_msm_downsampled_->AddPointCloudOptWithMSM(pcd_container_[i]);
      mesh_sm_filtered_->AddPointCloudOptWithSM(pcd_container_[i]);
      mesh_msm_filtered_->AddPointCloudOptWithMSM(pcd_container_[i]);
    }

    mesh_sm_filtered_->FilterMesh();
    mesh_msm_filtered_->FilterMesh();

    mesh_sm_downsampled_->DownsampleMesh();
    mesh_msm_downsampled_->DownsampleMesh();
  }

  std::vector<float> PCDContainer::GetXYZValuesOptWithSM(glm::mat4 curr_pose) {
    return mesh_sm_downsampled_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PCDContainer::GetRGBOptWithSMValues() {
    return mesh_sm_downsampled_->GetRGBValues();
  }

  std::vector<float> PCDContainer::GetXYZValuesOptWithMSM(glm::mat4 curr_pose) {
    return mesh_msm_downsampled_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PCDContainer::GetRGBOptWithMSMValues() {
    return mesh_msm_downsampled_->GetRGBValues();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCDContainer::GetFTFSMMeshPCDFile() {
    return mesh_sm_filtered_->GetPCDFile();
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCDContainer::GetMFSMMeshPCDFile() {
    return mesh_msm_filtered_->GetPCDFile();
  }

  void PCDContainer::ResetPCD() {
    while(!mesh_->Reset() || !mesh_msm_downsampled_->Reset() || !mesh_msm_downsampled_->Reset()) {
      LOGE("mesh is still running");
    }
    pcd_container_.clear();
    LOGE("MESH abd CONATINER are reseted");
  }
} // namespace rgb_depth_syn