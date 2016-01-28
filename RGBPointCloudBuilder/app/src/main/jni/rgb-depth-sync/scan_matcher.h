//
// Created by anastasia on 12.01.16.
//

#ifndef RGBPOINTCLOUDBUILDER_SCANMATCHER_H
#define RGBPOINTCLOUDBUILDER_SCANMATCHER_H

#include <Eigen/Geometry>
#include <vector>
#include <tango-gl/util.h>

#include "rgb-depth-sync/util.h"
#include <projectiveScanMatcher3d/projectiveScanMatcher3d.h>
#include <projectiveImage/sphericalProjectiveImage.h>

namespace rgb_depth_sync {
  class ScanMatcher {
    public:
      ScanMatcher();
      ~ScanMatcher();
      Eigen::Isometry3f Match(const std::vector<float> &frame_prev,
                              const std::vector<float> &frame_curr,
                              const Eigen::Isometry3f &odometryPose_prev,
                              const Eigen::Isometry3f &odometryPose_curr);
      void SetCameraIntrinsics(TangoCameraIntrinsics intrinsics);
    private:
      ProjectiveScanMatcher3d* projective_scan_matcher_;
      SphericalProjectiveImage* projective_image_;
      TangoCameraIntrinsics depth_camera_intrinsics_;
  };
}



#endif //RGBPOINTCLOUDBUILDER_SCANMATCHER_H
