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
    scan_matcher_count_ = 0;
    features_machter_count_ = 0;
    feature_matcher_ = cv::BFMatcher(cv::NORM_HAMMING);
  }

  LoopClosureDetector::~LoopClosureDetector() {


  }

  void LoopClosureDetector::Compute3(int lastIndex) {
    std::vector<NeighborWithDistance> orderedNeighbors;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    int orb_dist_th = 64;

    for (int i = 0; i < lastIndex; i++) {

      curr_rotation_ = glm::quat_cast((*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPose());
      int noOfGoodMatches = 0;

      if(abs(lastIndex - i) > 10) {

        prev_rotation_ = glm::quat_cast((*(pcd_container_->GetPCDContainer()))[i]->GetPose());

        rotation_distance_ = 1 - pow((curr_rotation_.w * prev_rotation_.w +
                                      curr_rotation_.x * prev_rotation_.x +
                                      curr_rotation_.y * prev_rotation_.y +
                                      curr_rotation_.z * prev_rotation_.z), 2);

        translation_distance_ = 100 * GetDistance(
            (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetTranslation(),
            (*(pcd_container_->GetPCDContainer()))[i]->GetTranslation());

        /*if (translation_distance_ <= 20 && translation_distance_ > 10) {
          features_machter_count_++;
          if ( (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrameDescriptors().type() == (*(pcd_container_->GetPCDContainer()))[i]->GetFrameDescriptors().type() &&
               (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrameDescriptors().cols == (*(pcd_container_->GetPCDContainer()))[i]->GetFrameDescriptors().cols ) {
            matches.clear();
            // returns the best match
            feature_matcher_.match((*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrameDescriptors(), (*(pcd_container_->GetPCDContainer()))[i]->GetFrameDescriptors(), matches);
            if (matches.size() <= 200)
              continue;
            for(int k = 0; k < matches.size(); k++) {
              if (matches[k].distance <= orb_dist_th) {
                noOfGoodMatches++;
                good_matches.push_back(matches[k]);
              }
              //LOGE("distance %f",matches[i].distance);
            }
            if (matches.size() - noOfGoodMatches <= 50) {
              /*cv::Mat outputImage;
              cv::drawMatches((*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrame(), (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrameKeyPoints(),
                              (*(pcd_container_->GetPCDContainer()))[i]->GetFrame(), (*(pcd_container_->GetPCDContainer()))[i]->GetFrameKeyPoints(),
                              matches, outputImage,
                              cv::Scalar(0, 0, 255),
                              cv::Scalar(255, 0, 0));
              NeighborWithDistance neighbor;
              neighbor.distance = translation_distance_;
              neighbor.id = i;
              neighbor.fm = true;
              orderedNeighbors.push_back(neighbor);
              /*char filename[1024];
              //LOGE("write file %i start...", pcd_file_counter_);
              sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/%i-%i.jpg", lastIndex, i);
              cv::imwrite(filename, outputImage);
              LOGE("%i %i noOfGoodMatches %i size %i", i, lastIndex, noOfGoodMatches, matches.size());
            }
          }
        }*/

        if (translation_distance_ <= 10) {
          //LOGE("scan matcher directly");
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance_;
          neighbor.id = i;
          orderedNeighbors.push_back(neighbor);
        }
      }

    }

    std::sort(orderedNeighbors.begin(), orderedNeighbors.end());

    for (int j = 0 ; j < orderedNeighbors.size() ; j++) {
      if (j > 0 && abs(orderedNeighbors[j-1].id - abs(orderedNeighbors[j].id) < 10))
        continue;
      distance_ = orderedNeighbors[j].distance;
      loop_closure_frame_idx_ = orderedNeighbors[j].id;

      //if (loop_closure_count_ % 2 == 0) {
      float overlap;
      /*Eigen::Isometry3f loop_pose = scan_matcher_->Match2(&overlap,
                                                 (*(pcd_container_->GetPCDContainer()))[loop_closure_frame_idx_]->GetPCD(),
                                                 (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPCD(),
                                                 (*(pcd_container_->GetPCDContainer()))[loop_closure_frame_idx_]->GetPose(),
                                                 (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPose());
      scan_matcher_count_++;

      float distance_after = 100*GetDistance(glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z()), glm::vec3(0,0,0));

      /*if (orderedNeighbors[j].fm == true) {
        Eigen::Quaternionf eigen_rot(loop_pose.rotation());
        glm::vec3 translation = glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z());
        glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());
        LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
        LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
        LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, lastIndex, orderedNeighbors[j].distance);
        LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, lastIndex, distance_after);
        LOGE("overlap: %f", overlap);
      }*/

      /*if(isnan(overlap))
        continue;

      //if (overlap >= 0.85 && std::fabs(distance_after - distance_) <= 5.0f) {
        Eigen::Quaternionf eigen_rot(loop_pose.rotation());
        glm::vec3 translation = glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z());
        glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());
        if (orderedNeighbors[j].fm == true) {
          LOGE("fm == true");
        }
        LOGE("vertices prev %i, curr %i, diff %i",
             (*(pcd_container_->GetPCDContainer()))[loop_closure_frame_idx_]->GetPCD()->points.size(),
             (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPCD()->points.size(),
             std::abs((*(pcd_container_->GetPCDContainer()))[loop_closure_frame_idx_]->GetPCD()->points.size() - (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPCD()->points.size()
             ));
        //LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
        //LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
        LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, lastIndex, orderedNeighbors[j].distance);
        LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, lastIndex, distance_after);
        LOGE("overlap: %f", overlap);
        loop_closure_poses_.insert(std::pair<key, value>(key(loop_closure_frame_idx_, lastIndex), value(distance_, loop_pose)));
        // break after one good match. Would be better if more than one match.
      //}*/
    }
  }

  void LoopClosureDetector::Compute2(int lastIndex) {
    std::vector<NeighborWithDistance> orderedNeighbors;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    int orb_dist_th = 64;

    for (int i = 0; i < lastIndex; i++) {

      curr_rotation_ = glm::quat_cast((*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPose());
      int noOfGoodMatches = 0;

      if(abs(lastIndex - i) > 10) {

        prev_rotation_ = glm::quat_cast((*(pcd_container_->GetPCDContainer()))[i]->GetPose());

        rotation_distance_ = 1 - pow((curr_rotation_.w * prev_rotation_.w +
                                      curr_rotation_.x * prev_rotation_.x +
                                      curr_rotation_.y * prev_rotation_.y +
                                      curr_rotation_.z * prev_rotation_.z), 2);

        translation_distance_ = 100 * GetDistance(
            (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetTranslation(),
            (*(pcd_container_->GetPCDContainer()))[i]->GetTranslation());

        if (translation_distance_ <= 20 && translation_distance_ > 10) {
          if ( (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrameDescriptors().type() == (*(pcd_container_->GetPCDContainer()))[i]->GetFrameDescriptors().type() &&
              (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrameDescriptors().cols == (*(pcd_container_->GetPCDContainer()))[i]->GetFrameDescriptors().cols ) {
            matches.clear();
            // returns the best match
            feature_matcher_.match((*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrameDescriptors(), (*(pcd_container_->GetPCDContainer()))[i]->GetFrameDescriptors(), matches);
            if (matches.size() <= 400)
              continue;
            for(int k = 0; k < matches.size(); k++) {
              if (matches[k].distance <= orb_dist_th) {
                noOfGoodMatches++;
                good_matches.push_back(matches[k]);
              }
              //LOGE("distance %f",matches[i].distance);
            }
            if (matches.size() - noOfGoodMatches <= 150) {
              /*cv::Mat outputImage;
              cv::drawMatches((*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrame(), (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetFrameKeyPoints(),
                              (*(pcd_container_->GetPCDContainer()))[i]->GetFrame(), (*(pcd_container_->GetPCDContainer()))[i]->GetFrameKeyPoints(),
                              matches, outputImage,
                              cv::Scalar(0, 0, 255),
                              cv::Scalar(255, 0, 0));*/
              NeighborWithDistance neighbor;
              neighbor.distance = translation_distance_;
              neighbor.id = i;
              neighbor.fm = true;
              orderedNeighbors.push_back(neighbor);
              /*char filename[1024];
              //LOGE("write file %i start...", pcd_file_counter_);
              sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/%i-%i.jpg", lastIndex, i);
              cv::imwrite(filename, outputImage);*/
              LOGE("%i %i noOfGoodMatches %i size %i", i, lastIndex, noOfGoodMatches, matches.size());
            }
          }
        }

        if (translation_distance_ <= 10) {
          //LOGE("scan matcher directly");
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance_;
          neighbor.id = i;
          orderedNeighbors.push_back(neighbor);
        }
      }
    }

    std::sort(orderedNeighbors.begin(), orderedNeighbors.end());

    for (int j = 0 ; j < orderedNeighbors.size() ; j++) {
      if (j > 0 && abs(orderedNeighbors[j-1].id - abs(orderedNeighbors[j].id) < 10))
        continue;
      distance_ = orderedNeighbors[j].distance;
      loop_closure_frame_idx_ = orderedNeighbors[j].id;

      //if (loop_closure_count_ % 2 == 0) {
      float overlap;
      Eigen::Isometry3f loop_pose = scan_matcher_->Match(&overlap,
                                                         (*(pcd_container_->GetPCDContainer()))[loop_closure_frame_idx_]->GetXYZValues(),
                                                         (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetXYZValues(),
                                                         (*(pcd_container_->GetPCDContainer()))[loop_closure_frame_idx_]->GetPose(),
                                                         (*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPose());

      float distance_after = 100*GetDistance(glm::vec3(0,0,0), glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z()));

      /*if (orderedNeighbors[j].fm == true) {
        Eigen::Quaternionf eigen_rot(loop_pose.rotation());
        glm::vec3 translation = glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z());
        glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());
        LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
        LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
        LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, lastIndex, orderedNeighbors[j].distance);
        LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, lastIndex, distance_after);
        LOGE("overlap: %f", overlap);
      }*/


      if(isnan(overlap))
        continue;

      if (overlap >= 0.85) {
        Eigen::Quaternionf eigen_rot(loop_pose.rotation());
        glm::vec3 translation = glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z());
        glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());
        if (orderedNeighbors[j].fm == true) {
          LOGE("fm == true");
        }
        LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
        LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
        LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, lastIndex, orderedNeighbors[j].distance);
        LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, lastIndex, distance_after);
        LOGE("overlap: %f", overlap);
        loop_closure_poses_.insert(std::pair<key, value>(key(loop_closure_frame_idx_, lastIndex), value(distance_, loop_pose)));
        // break after one good match. Would be better if more than one match.
      }
    }

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

  glm::vec4 LoopClosureDetector::ConvertDepthPointTo3DPoint(const glm::vec3 &depth_point) {
    glm::vec4 pcd_point;
    pcd_point[0] = static_cast<float>(depth_point.z * (depth_point.x - color_camera_intrinsics_.cx) / color_camera_intrinsics_.fx);
    pcd_point[1] = static_cast<float>(depth_point.z * (depth_point.y - color_camera_intrinsics_.cy) / color_camera_intrinsics_.fy);
    pcd_point[2] = depth_point.z;
    pcd_point[3] = 1.0f;
    return pcd_point;
  }
}