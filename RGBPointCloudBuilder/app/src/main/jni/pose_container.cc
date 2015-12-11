//
// Created by anastasia on 02.12.15.
//

#include "rgb-depth-sync/pose_container.h"

namespace rgb_depth_sync {

  PoseContainer::PoseContainer() {
    //pcl_container_ = new std::vector<float>;
  }

  PoseContainer::~PoseContainer(){}

  void PoseContainer::SetPose(const TangoPoseData* pose) {
    current_pose_ = *pose;
    pose_container_[current_pose_.timestamp] = current_pose_;
  }

  const std::map<double, TangoPoseData > PoseContainer::GetPoseContainer() const {
    return pose_container_;
  }

  void PoseContainer::UpdateContainer() {

    TangoPoseData pose_start_service_T_device_t1;

    TangoCoordinateFramePair frame_pair;
    frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;

    std::map<double, TangoPoseData >::iterator it;
    it = pose_container_.begin();

    while(it != pose_container_.end()) {
      TangoService_getPoseAtTime(it->first, frame_pair, &it->second);
      it++;
    }

  }

  TangoPoseData PoseContainer::GetCurrentPose() {
    return current_pose_;
  }

  TangoPoseData* PoseContainer::GetPoseAtTime(double timestamp) {
    std::map<double, TangoPoseData >::iterator it, it_upper_bound;
    it = pose_container_.find(timestamp);

    if (it != pose_container_.end()) {
      return &it->second;
    } else {
      it_upper_bound = pose_container_.upper_bound(timestamp);
      if (it_upper_bound != pose_container_.end() && it_upper_bound->first - timestamp < 1.0) {
        //LOGE("timestamp: %lf, ~timestamp: %lf", it_upper_bound->first , timestamp);
        return &it_upper_bound->second;
      } else {
        return nullptr;
      }
    }
  }



} // namespace rgb_depth_sync