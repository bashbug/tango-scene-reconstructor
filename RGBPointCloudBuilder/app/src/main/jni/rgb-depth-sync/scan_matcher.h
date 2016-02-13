//
// Created by anastasia on 12.01.16.
//

#ifndef RGBPOINTCLOUDBUILDER_SCANMATCHER_H
#define RGBPOINTCLOUDBUILDER_SCANMATCHER_H

#include <vector>
#include <Eigen/Geometry>
#include <tango_client_api.h>
#include <tango-gl/util.h>
#include <projectiveScanMatcher3d/projectiveScanMatcher3d.h>
#include <projectiveImage/sphericalProjectiveImage.h>
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {
  class ScanMatcher {
    public:
      ScanMatcher();
      ~ScanMatcher();
      Eigen::Isometry3f Match(const ProjectiveImage::ImagePixels& frame_prev,
                              const ProjectiveImage::ImagePixels& frame_curr,
                              const glm::mat4& odometryPose_prev,
                              const glm::mat4& odometryPose_curr);
      Eigen::Isometry3f Match2(const std::vector<float>& frame_prev,
                              const std::vector<float>& frame_curr,
                              const glm::mat4& odometryPose_prev,
                              const glm::mat4& odometryPose_curr);
    private:
      TangoCameraIntrinsics color_camera_intrinsics_;
  };
}



#endif //RGBPOINTCLOUDBUILDER_SCANMATCHER_H
