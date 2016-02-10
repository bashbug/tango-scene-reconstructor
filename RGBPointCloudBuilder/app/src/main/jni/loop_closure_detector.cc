//
// Created by anastasia on 02.02.16.
//

#include "rgb-depth-sync/loop_closure_detector.h"

namespace rgb_depth_sync {

  LoopClosureDetector::LoopClosureDetector(PCDContainer* pcd_container) {
    pcd_container_ = pcd_container;
    max_loop_closure_per_frame_ = 2;

    bf_matcher_ = new cv::BFMatcher(cv::NORM_HAMMING, false); // with true : usually produces best results with minimal number of outliers when there are enough matches. This is alternative to the ratio test
    orb_ = cv::ORB::create(1000);

    TangoErrorType ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_COLOR, &color_camera_intrinsics_);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to get the intrinsics for the color camera.");
    }

    curr_depth_point_lu_ = glm::vec3(119.0f, 119.0f, 0.5f);
    curr_pcd_point_lu_ = ConvertDepthPointTo3DPoint(curr_depth_point_lu_);
    curr_depth_point_ld_ = glm::vec3(119.0f, 599.0f, 0.5f);
    curr_pcd_point_ld_ = ConvertDepthPointTo3DPoint(curr_depth_point_ld_);
    curr_depth_point_cc_ = glm::vec3(639.0f, 359.0f, 0.5f);
    curr_pcd_point_cc_ = ConvertDepthPointTo3DPoint(curr_depth_point_cc_);
    curr_depth_point_ru_ = glm::vec3(1159.0f, 119.0f, 0.5f);
    curr_pcd_point_ru_ = ConvertDepthPointTo3DPoint(curr_depth_point_ru_);
    curr_depth_point_rd_ = glm::vec3(1159.0f, 599.0f, 0.5f);
    curr_pcd_point_rd_ = ConvertDepthPointTo3DPoint(curr_depth_point_rd_);

    img_file_writer_ = new rgb_depth_sync::IMGFileWriter();

    distance_ = 0;
    distance_accum_ = 0;
    distance_accum_counter_ = 0;

  }

  LoopClosureDetector::~LoopClosureDetector() {

  }

  void LoopClosureDetector::Compute(int lastIndex) {

    const std::vector<PCD*>& pcd_container_data = *(pcd_container_->GetPCDContainer());

    curr_rotation_ = glm::quat_cast(pcd_container_data[lastIndex]->GetPose());
    counter_loop_closure_per_frame_ = 0;

    for (int i = 0; i < lastIndex; i++) {

      if(abs(lastIndex - i) > 10){

        prev_rotation_ = glm::quat_cast(pcd_container_data[i]->GetPose());

        rotation_distance_ = 1 - pow((curr_rotation_.w*prev_rotation_.w + curr_rotation_.x*prev_rotation_.x + curr_rotation_.y*prev_rotation_.y + curr_rotation_.z*prev_rotation_.z),2);

        translation_distance_ = 100.0f*GetDistance(pcd_container_data[lastIndex]->GetTranslation(), pcd_container_data[i]->GetTranslation());

        if (translation_distance_ <= 5 && rotation_distance_ < 0.05) {
          LOGE("%i : %i trans_dist : %f, rot_dist : %f", lastIndex, i, translation_distance_, rotation_distance_);
          ScanMatcher scan_matcher;
          std::future <Eigen::Isometry3f> loop_pose_async = std::async(std::launch::async,
                                                                       &rgb_depth_sync::ScanMatcher::Match,
                                                                       scan_matcher,
                                                                       pcd_container_data[i]->GetImagePixels(),
                                                                       pcd_container_data[lastIndex]->GetImagePixels(),
                                                                       pcd_container_data[i]->GetPose(),
                                                                       pcd_container_data[lastIndex]->GetPose());
          int dist = 100 - translation_distance_;

          loop_closure_poses_.insert(std::pair<key, value>(key(i, lastIndex), value(dist, std::move(loop_pose_async))));
          if (counter_loop_closure_per_frame_ >= max_loop_closure_per_frame_) {
            break;
          }
        }
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