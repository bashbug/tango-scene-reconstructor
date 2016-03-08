#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  PCDContainer::PCDContainer(std::shared_ptr<std::mutex> pcd_mtx,
                             std::shared_ptr<std::condition_variable> consume_pcd) {
    pcd_mtx_ = pcd_mtx;
    consume_pcd_ = consume_pcd;
    mesh_ = new rgb_depth_sync::Mesh();
  }

  PCDContainer::~PCDContainer(){
  }

  void PCDContainer::AddPCD(PCD *pcd) {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);
    pcd_container_.push_back(pcd);
    mesh_->AddPointCloud(pcd);
    consume_pcd_->notify_one();
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

  void PCDContainer::ResetPCD() {
    pcd_container_.clear();
  }
} // namespace rgb_depth_syn