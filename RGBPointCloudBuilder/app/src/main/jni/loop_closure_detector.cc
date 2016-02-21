//
// Created by anastasia on 02.02.16.
//

#include "rgb-depth-sync/loop_closure_detector.h"

namespace rgb_depth_sync {

  LoopClosureDetector::LoopClosureDetector(PCDContainer* pcd_container) {
    pcd_container_ = pcd_container;

    TangoErrorType ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_COLOR, &color_camera_intrinsics_);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to get the intrinsics for the color camera.");
    }
    distance_ = 0;
  }

  LoopClosureDetector::~LoopClosureDetector() {

  }

  void LoopClosureDetector::Compute(int lastIndex) {

    bool first = true;
    std::vector<NeighborWithDistance> orderedNeighbors;

    curr_rotation_ = glm::quat_cast((*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPose());

    for (int i = 0; i < lastIndex; i++) {

      if(abs(lastIndex - i) > 10) {

        prev_rotation_ = glm::quat_cast((*(pcd_container_->GetPCDContainer()))[i]->GetPose());

        rotation_distance_ = 1 - pow((curr_rotation_.w * prev_rotation_.w +
                                      curr_rotation_.x * prev_rotation_.x +
                                      curr_rotation_.y * prev_rotation_.y +
                                      curr_rotation_.z * prev_rotation_.z), 2);

        translation_distance_ = 100 * GetDistance(
            (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetTranslation(),
            (*(pcd_container_->GetPCDContainer()))[i]->GetTranslation());

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
                                                         (*(pcd_container_->GetPCDContainer()))[loop_closure_frame_idx_]->GetXYZValues(),
                                                         (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetXYZValues(),
                                                         (*(pcd_container_->GetPCDContainer()))[loop_closure_frame_idx_]->GetPose(),
                                                         (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPose());

      float distance_after = 100*GetDistance(glm::vec3(0,0,0), glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z()));

      if (overlap >= 0.85 && (distance_after < orderedNeighbors[j].distance)) {
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

  glm::vec4 LoopClosureDetector::ConvertDepthPointTo3DPoint(const glm::vec3 &depth_point) {
    glm::vec4 pcd_point;
    pcd_point[0] = static_cast<float>(depth_point.z * (depth_point.x - color_camera_intrinsics_.cx) / color_camera_intrinsics_.fx);
    pcd_point[1] = static_cast<float>(depth_point.z * (depth_point.y - color_camera_intrinsics_.cy) / color_camera_intrinsics_.fy);
    pcd_point[2] = depth_point.z;
    pcd_point[3] = 1.0f;
    return pcd_point;
  }
}