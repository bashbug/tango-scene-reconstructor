#ifndef RGBPOINTCLOUDBUILDER_LOOPCLOSUREDETECTOR_H
#define RGBPOINTCLOUDBUILDER_LOOPCLOSUREDETECTOR_H

#include <map>
#include <utility>
#include <Eigen/Geometry>
#include <tango_client_api.h>
#include <tango-gl/util.h>
#include "rgb-depth-sync/pcd_container.h"
#include "rgb-depth-sync/scan_matcher.h"

namespace rgb_depth_sync {

  struct NeighborWithDistance {
    float distance;
    int id;
    bool fm = false;
    bool operator < (const NeighborWithDistance& other) const { return distance < other.distance; }
  };

  typedef std::pair<int, int> key;
  typedef std::pair<int, Eigen::Isometry3f > value;

  class LoopClosureDetector {
  public:
    LoopClosureDetector(PCDContainer* pcd_container);
    ~LoopClosureDetector();
    void Compute(int lastIndex);
    void GetLoopClosurePoses(std::map<key, value>** loop_closure_poses);

  private:
    float GetDistance(const glm::vec3 &curr_trans, const glm::vec3 &prev_trans);
    float GetPoseDistance(const glm::vec3& curr_translation, const glm::quat& curr_rotation,
                          const glm::vec3& prev_translation, const glm::quat& prev_rotation);
    float rotation_distance_;
    float translation_distance_;
    int loop_closure_frame_idx_;
    int scan_matcher_count_;
    int features_machter_count_;
    glm::quat curr_rotation_;
    glm::quat prev_rotation_;
    PCDContainer* pcd_container_;
    std::map<key, value> loop_closure_poses_;
    float distance_;
    ScanMatcher* scan_matcher_;
  };
}


#endif //RGBPOINTCLOUDBUILDER_LOOPCLOSUREDETECTOR_H
