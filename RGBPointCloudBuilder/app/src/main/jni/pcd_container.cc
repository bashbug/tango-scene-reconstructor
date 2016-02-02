#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  PCDContainer::PCDContainer(std::mutex *pcd_mtx, std::condition_variable *consume_pcd) {
    //pcd_mtx_ = *pcd_mtx;
    //consume_pcd_ = *consume_pcd;
  }

  PCDContainer::~PCDContainer(){
  }

  void PCDContainer::AddPCD(PCD *pcd) {
    std::unique_lock<std::mutex> lock(pcd_mtx_);
    pcd_container_.push_back(pcd);
  }

  PCD* PCDContainer::GetLatestPCD() {
    std::unique_lock<std::mutex> lock(pcd_mtx_);
    if (pcd_container_.empty()) {
      return nullptr;
    } else {
      return pcd_container_.back();
    }
  }

  int PCDContainer::GetPCDContainerLastIndex() {
    std::unique_lock<std::mutex> lock(pcd_mtx_);
    return pcd_container_.size();
  }

} // namespace rgb_depth_syn