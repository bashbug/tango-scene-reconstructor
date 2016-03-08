//
// Created by anastasia on 02.02.16.
//

#include "rgb-depth-sync/loop_closure_detector.h"

namespace rgb_depth_sync {

  LoopClosureDetector::LoopClosureDetector(PCDContainer* pcd_container) {
    pcd_container_ = pcd_container;
    distance_ = 0;
    scan_matcher_ = new rgb_depth_sync::ScanMatcher();
  }

  LoopClosureDetector::~LoopClosureDetector() {}

  void LoopClosureDetector::Compute(int lastIndex) {

    bool first = true;
    std::vector<NeighborWithDistance> orderedNeighbors;

    curr_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[lastIndex]->GetPose());

    for (int i = 0; i < lastIndex; i++) {

      if(abs(lastIndex - i) > 10) {

        prev_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[i]->GetPose());

        rotation_distance_ = 1 - pow((curr_rotation_.w * prev_rotation_.w +
                                      curr_rotation_.x * prev_rotation_.x +
                                      curr_rotation_.y * prev_rotation_.y +
                                      curr_rotation_.z * prev_rotation_.z), 2);

        translation_distance_ = 100 * GetDistance(
            pcd_container_->pcd_container_[lastIndex]->GetTranslation(),
            pcd_container_->pcd_container_[i]->GetTranslation());

        if (translation_distance_ <= 10) {
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance_;
          neighbor.id = i;
          orderedNeighbors.push_back(neighbor);
        }
      }
    }

    for (int j = 0 ; j < orderedNeighbors.size() ; j++) {
      std::sort(orderedNeighbors.begin(), orderedNeighbors.end());
      if (j > 0 && abs(orderedNeighbors[j-1].id - abs(orderedNeighbors[j].id) < 10))
        continue;
      distance_ = 1;
      loop_closure_frame_idx_ = orderedNeighbors[j].id;

      //if (loop_closure_count_ % 2 == 0) {
      float overlap;
      Eigen::Isometry3f loop_pose = scan_matcher_->Match(&overlap,
                                                         pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetXYZValues(),
                                                         pcd_container_->pcd_container_[lastIndex]->GetXYZValues(),
                                                         pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPose(),
                                                         pcd_container_->pcd_container_[lastIndex]->GetPose());

      float distance_after = 100*GetDistance(glm::vec3(0,0,0), glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z()));

      if(isnan(overlap))
        continue;

      if (overlap >= 0.85) {
        Eigen::Quaternionf eigen_rot(loop_pose.rotation());
        glm::vec3 translation = glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z());
        glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());
        LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
        LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
        LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, lastIndex, orderedNeighbors[j].distance);
        LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, lastIndex, distance_after);
        LOGE("overlap: %f", overlap);
        loop_closure_poses_.insert(std::pair<key, value>(key(loop_closure_frame_idx_, lastIndex), value(distance_, loop_pose)));
      }
    }
  }

  void LoopClosureDetector::GetLoopClosurePoses(std::map<key, value>** loop_closure_poses) {
    *loop_closure_poses = &loop_closure_poses_;
    LOGE("scan matcher count: %i", scan_matcher_count_);
    LOGE("features machter count: %i", features_machter_count_);
  }

  float LoopClosureDetector::GetPoseDistance(const glm::vec3& curr_translation, const glm::quat& curr_rotation, const glm::vec3& prev_translation, const glm::quat& prev_rotation) {
    return sqrtf(static_cast<float>((curr_translation.x - prev_translation.x)*(curr_translation.x - prev_translation.x) +  // tx
                                    (curr_translation.y - prev_translation.y)*(curr_translation.y - prev_translation.y) + // ty
                                    (curr_translation.z - prev_translation.z)*(curr_translation.z - prev_translation.z) + // tz
                                    (curr_rotation.x - prev_rotation.x)*(curr_rotation.x - prev_rotation.x) + //qx
                                    (curr_rotation.y - prev_rotation.y)*(curr_rotation.y - prev_rotation.y) + //qy
                                    (curr_rotation.z - prev_rotation.z)*(curr_rotation.z - prev_rotation.z))); //qz
  }

  float LoopClosureDetector::GetDistance(const glm::vec3 &curr_depth_point, const glm::vec3 &trans_depth_point) {
    return sqrtf(static_cast<float>((curr_depth_point[0] - trans_depth_point[0])*(curr_depth_point[0] - trans_depth_point[0]) +
                                    (curr_depth_point[1] - trans_depth_point[1])*(curr_depth_point[1] - trans_depth_point[1]) +
                                    (curr_depth_point[2] - trans_depth_point[2])*(curr_depth_point[2] - trans_depth_point[2])));
  }

}