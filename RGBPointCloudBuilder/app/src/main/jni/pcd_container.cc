#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  PCDContainer::PCDContainer() {
    mesh_ = new rgb_depth_sync::Mesh();
    mesh_sm_ = new rgb_depth_sync::Mesh();
    mesh_msm_ = new rgb_depth_sync::Mesh();
  }

  PCDContainer::~PCDContainer(){
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
      mesh_sm_->AddPointCloudOptWithSM(pcd_container_[i]);
      mesh_msm_->AddPointCloudOptWithMSM(pcd_container_[i]);
    }
    mesh_sm_->DownsampleMesh();
    mesh_msm_->DownsampleMesh();
  }

  std::vector<float> PCDContainer::GetXYZValuesOptWithSM(glm::mat4 curr_pose) {
    return mesh_sm_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PCDContainer::GetRGBOptWithSMValues() {
    return mesh_sm_->GetRGBValues();
  }

  std::vector<float> PCDContainer::GetXYZValuesOptWithMSM(glm::mat4 curr_pose) {
    return mesh_msm_->GetXYZValues(curr_pose);
  }

  std::vector<uint8_t> PCDContainer::GetRGBOptWithMSMValues() {
    return mesh_msm_->GetRGBValues();
  }

  void PCDContainer::ResetPCD() {
    while(!mesh_->Reset()) {
      LOGE("mesh is still running");
    }
    pcd_container_.clear();
    LOGE("MESH abd CONATINER are reseted");
  }
} // namespace rgb_depth_syn