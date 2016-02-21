#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  PCDContainer::PCDContainer(std::shared_ptr<std::mutex> pcd_mtx, std::shared_ptr<std::condition_variable> consume_pcd) {
    pcd_mtx_ = pcd_mtx;
    consume_pcd_ = consume_pcd;
    resolution_ = 0.0001; // 0.01 cm => 0.0001m
  }

  PCDContainer::~PCDContainer(){
  }

  void PCDContainer::AddPCD(PCD *pcd) {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);
    pcd_container_.push_back(pcd);
    /*LOGE("ADD point start...");
    for (int i = 0; i < pcd->GetXYZValues().size(); i+=3) {
      Point p;
      p.x = pcd->GetXYZValues()[i];
      p.y = pcd->GetXYZValues()[i+1];
      p.z = pcd->GetXYZValues()[i+2];
      p.r = pcd->GetRGBValues()[i];
      p.g = pcd->GetRGBValues()[i+1];
      p.b = pcd->GetRGBValues()[i+2];
      xyz_rgb_map_[p.x/resolution_][p.y/resolution_][p.z/resolution_] = p;
      LOGE("new point: %f, %f, %f, %i, %i, %i", xyz_rgb_map_[p.x/resolution_][p.y/resolution_][p.z/resolution_].x,
           xyz_rgb_map_[p.x/resolution_][p.y/resolution_][p.z/resolution_].y, xyz_rgb_map_[p.x/resolution_][p.y/resolution_][p.z/resolution_].z,
           xyz_rgb_map_[p.x/resolution_][p.y/resolution_][p.z/resolution_].r, xyz_rgb_map_[p.x/resolution_][p.y/resolution_][p.z/resolution_].g, xyz_rgb_map_[p.x/resolution_][p.y/resolution_][p.z/resolution_].b);
    }
    LOGE("ADD point stop...");*/
    consume_pcd_->notify_one();
  }

  void PCDContainer::ResetPCD() {
    pcd_container_.clear();
  }

  PCD* PCDContainer::GetLatestPCD() {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);
    if (pcd_container_.empty()) {
      return nullptr;
    } else {
      return pcd_container_.back();
    }
  }

  std::pair<std::vector<float>, std::vector<uint8_t> > PCDContainer::GetXYZRGBValues() {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);
    std::map<int, std::map<int, std::map <int, Point> > >::iterator it_x;
    std::map<int, std::map <int, Point> >::iterator it_y;
    std::map <int, Point>::iterator it_z;
    xyz_.clear();
    rgb_.clear();
    LOGE("GET XYZRGB start...");
    for (it_x = xyz_rgb_map_.begin(); it_x != xyz_rgb_map_.end(); it_x++) {
      for (it_y = it_x->second.begin(); it_y != it_x->second.end(); it_y++) {
        for (it_z = it_y->second.begin(); it_z != it_y->second.end(); it_z++) {
          LOGE("%i, %i, %i, %f, %f, %f, %i, %i, %i", it_x->first, it_y->first, it_z->first, it_z->second.x, it_z->second.y, it_z->second.z, it_z->second.r, it_z->second.g, it_z->second.b);
          xyz_.push_back(it_z->second.x);
          xyz_.push_back(it_z->second.y);
          xyz_.push_back(it_z->second.z);
          rgb_.push_back(it_z->second.r);
          rgb_.push_back(it_z->second.g);
          rgb_.push_back(it_z->second.b);
        }
      }
    }
    LOGE("GET XYZRGB stop...");
    return std::pair<std::vector<float>, std::vector<uint8_t> >(xyz_, rgb_);
  }

  int PCDContainer::GetPCDContainerLastIndex() {
    return pcd_container_.size()-1;
  }

  std::vector<PCD*>* PCDContainer::GetPCDContainer() {
    return &pcd_container_;
  }
} // namespace rgb_depth_syn