//
// Created by anastasia on 09.12.15.
//

#ifndef RGBPOINTCLOUDBUILDER_POINT_H
#define RGBPOINTCLOUDBUILDER_POINT_H

#include "tango-gl/util.h"
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  class Point {
    public:
      Point(glm::vec3 point, TangoPoseData* pose_pointer, std::vector<uint8_t>& rgb);
      ~Point();
      glm::vec3 point_;
      TangoPoseData* pose_pointer_;
      // copy current pose pointer, to backtransform point before updating with new pose
      glm::vec3 current_translation_;
      glm::quat current_rotation_;
      std::vector<uint8_t> rgb_;
      void UpdatePoint();
    private:
      void TransformPoint();
      void UpdatePose();
      void Set();
  };

}
#endif //RGBPOINTCLOUDBUILDER_POINT_H
