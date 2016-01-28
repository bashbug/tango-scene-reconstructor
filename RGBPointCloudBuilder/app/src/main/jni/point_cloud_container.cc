#include "rgb-depth-sync/point_cloud_container.h"
#include <tango-gl/conversions.h>
#include <Eigen/Dense>

namespace rgb_depth_sync {

  PointCloudContainer::PointCloudContainer() {
    //merged_point_cloud_xyz_values.reserve()
  }

  PointCloudContainer::~PointCloudContainer(){
    LOGE("PointCloudContainer is destroyed...");
  }

  /*PointCloudContainer::Compare(std::vector<float> xyz_1, std::vector<float> xyz_2) {
    if ()
  }*/

  void PointCloudContainer::SetPointCloudData(PointCloudData* pcd) {
    pcd_container_.push_back(pcd);
    size_t current_size = merged_point_cloud_xyz_values_.size();
    /*std::vector<float> xyz_tmp = pcd->GetXYZValues();
    std::vector<uint8_t> rgb_tmp = pcd->GetRGBValues();
    merged_point_cloud_xyz_values.insert(merged_point_cloud_xyz_values.end(), xyz_tmp.begin(), xyz_tmp.end());
    merged_point_cloud_xyz_values.insert(merged_point_cloud_xyz_values.end(), xyz_tmp.begin(), xyz_tmp.end());*/
    /*merged_point_cloud_xyz_values_.resize(merged_point_cloud_xyz_values_.size() + pcd->GetXYZValues().size());
    memcpy(&merged_point_cloud_xyz_values_[current_size], pcd->GetXYZValues().data(), pcd->GetXYZValues().size());

    merged_point_cloud_rgb_values_.resize(merged_point_cloud_rgb_values_.size() + pcd->GetRGBValues().size());
    memcpy(&merged_point_cloud_rgb_values_[current_size], pcd->GetRGBValues().data(), pcd->GetRGBValues().size());*/
  }

  PointCloudData* PointCloudContainer::GetLatestPCD() {
    //LOGE("PointCloudContainer : GetLatestPCD start...");
    if (pcd_container_.empty()) {
      //LOGE("PointCloudContainer : GetLatestPCD is empty...");
      return nullptr;
    } else {
      //LOGE("PointCloudContainer : GetLatestPCD is not empty...");
      return pcd_container_.back();
    }
  }

} // namespace rgb_depth_syn