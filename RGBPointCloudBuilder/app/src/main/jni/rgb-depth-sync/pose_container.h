//
// Created by anastasia on 02.12.15.
//

#ifndef RGBPOINTCLOUDBUILDER_POSE_CONTAINER_H
#define RGBPOINTCLOUDBUILDER_POSE_CONTAINER_H

#include <map>
#include "tango-gl/util.h"
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  class PoseContainer {
    public:
      explicit PoseContainer();
      ~PoseContainer();
      void SetPose(const TangoPoseData* pose);
      void UpdateContainer();
      const std::map<double, TangoPoseData > GetPoseContainer() const;
      TangoPoseData* GetPoseAtTime(double timestamp);
      TangoPoseData GetCurrentPose();
    private:
      std::map<double, TangoPoseData> pose_container_;
      TangoPoseData current_pose_;
  };

} // namespace rgb_depth_sync


#endif //RGBPOINTCLOUDBUILDER_POSE_CONTAINER_H
