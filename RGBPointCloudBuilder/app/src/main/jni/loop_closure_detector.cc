//
// Created by anastasia on 02.02.16.
//

#include "rgb-depth-sync/loop_closure_detector.h"

/***
  * depth camera width: 320
  * depth camera height: 180
  * depth camera fx: 260.714000
  * depth camera fy: 260.769000
  * depth camera cx: 161.307000
  * depth camera cy: 88.500700
***/

namespace rgb_depth_sync {

  LoopClosureDetector::LoopClosureDetector(PCDContainer* pcd_container) {
    pcd_container_ = pcd_container;
    distance_ = 0;
    scan_matcher_ = new rgb_depth_sync::ScanMatcher();
    feature_matcher_ = cv::BFMatcher(cv::NORM_HAMMING);
    orb_dist_th_ = 64;
  }

  LoopClosureDetector::~LoopClosureDetector() {}

  void LoopClosureDetector::Compute(int lastIndex) {

    bool first = true;
    std::vector<NeighborWithDistance> orderedNeighbors;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    int noOfGoodMatches = 0;
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

        if (translation_distance_ <= 20 && translation_distance_ > 10) {

          std::clock_t start_fm = std::clock();

          if ( (pcd_container_->pcd_container_[lastIndex]->GetFrameDescriptors().type() ==
                    pcd_container_->pcd_container_[i]->GetFrameDescriptors().type()) &&
               (pcd_container_->pcd_container_[lastIndex]->GetFrameDescriptors().cols ==
                   pcd_container_->pcd_container_[i]->GetFrameDescriptors().cols) ) {

            matches.clear();
            // returns the best match
            feature_matcher_.match(pcd_container_->pcd_container_[lastIndex]->GetFrameDescriptors(),
                                   pcd_container_->pcd_container_[i]->GetFrameDescriptors(), matches);

            if (matches.size() <= 200)
              continue;

            for(int k = 0; k < matches.size(); k++) {
              if (matches[k].distance <= orb_dist_th_) {
                noOfGoodMatches++;
                good_matches.push_back(matches[k]);
              }
              //LOGE("distance %f",matches[i].distance);
            }

            if (matches.size() - noOfGoodMatches <= 50) {

              NeighborWithDistance neighbor;
              neighbor.distance = translation_distance_;
              neighbor.id = i;
              neighbor.fm = true;
              neighbor.no_matches = noOfGoodMatches;
              orderedNeighbors.push_back(neighbor);
              int diff = (std::clock() - start_fm) / (double)(CLOCKS_PER_SEC / 1000);
              LOGE("Feature matcher  ------- time %i", diff);
              /*cv::Mat outputImage;
              cv::drawMatches(pcd_container_->pcd_container_[lastIndex]->GetFrame(),
                              pcd_container_->pcd_container_[lastIndex]->GetFrameKeyPoints(),
                              pcd_container_->pcd_container_[i]->GetFrame(),
                              pcd_container_->pcd_container_[i]->GetFrameKeyPoints(),
                              matches, outputImage,
                              cv::Scalar(0, 0, 255),
                              cv::Scalar(255, 0, 0));

              char filename[1024];
              sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/poss-%i-%i.jpg", lastIndex, i);
              cv::imwrite(filename, outputImage);
              LOGE("%i %i noOfGoodMatches %i size %i", i, loop_closure_frame_idx_, noOfGoodMatches, matches.size());*/
            }
          }
        }

        if (translation_distance_ <= 10) {
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance_;
          neighbor.id = i;
          orderedNeighbors.push_back(neighbor);
        }
      }
    }

    float overlap = 0;
    for (int j = 0 ; j < orderedNeighbors.size() ; j++) {
      std::sort(orderedNeighbors.begin(), orderedNeighbors.end());
      // if we already have a good matche continue. If last match has not an overlap >= 0.85 try next
      if (j > 0 && abs(orderedNeighbors[j-1].id - abs(orderedNeighbors[j].id) < 10))
        continue;

      distance_ = orderedNeighbors[j].distance;
      loop_closure_frame_idx_ = orderedNeighbors[j].id;

      //if (loop_closure_count_ % 2 == 0) {
      std::clock_t start = std::clock();
      Eigen::Isometry3f loop_pose = scan_matcher_->Match(&overlap,
                                                         pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetXYZValues(),
                                                         pcd_container_->pcd_container_[lastIndex]->GetXYZValues(),
                                                         pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPose(),
                                                         pcd_container_->pcd_container_[lastIndex]->GetPose());

      float distance_after = 100*GetDistance(glm::vec3(0,0,0), glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z()));

      if(isnan(overlap))
        continue;

      if (orderedNeighbors[j].fm == true) {
        LOGE("Feature matcher overlap: %f", overlap);
        /*cv::Mat outputImage;
        cv::drawMatches(pcd_container_->pcd_container_[lastIndex]->GetFrame(),
                        pcd_container_->pcd_container_[lastIndex]->GetFrameKeyPoints(),
                        pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetFrame(),
                        pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetFrameKeyPoints(),
                        matches, outputImage,
                        cv::Scalar(0, 0, 255),
                        cv::Scalar(255, 0, 0));

        char filename[1024];
        sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/%i-%i.jpg", lastIndex, loop_closure_frame_idx_);
        cv::imwrite(filename, outputImage);
        LOGE("%i %i noOfGoodMatches %i size %i", lastIndex, loop_closure_frame_idx_, orderedNeighbors[j].no_matches, matches.size());*/
      }

      if (overlap >= 0.85 && std::fabs(distance_after - distance_) <= 5.0f) {
        LOGE("GOOD LOOP");
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
        int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
        LOGE("Scan matcher  ---------- time %i", diff);
      } else {
        LOGE("BAD LOOP");
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
        int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
        LOGE("Scan matcher  ---------- time %i", diff);
      }
    }
  }

  void LoopClosureDetector::Compute2(int lastIndex) {

    bool first = true;
    std::vector<NeighborWithDistance> orderedNeighbors;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    int noOfGoodMatches = 0;
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

        if (translation_distance_ <= 20 && translation_distance_ > 10) {

          std::clock_t start_fm = std::clock();

          if ( (pcd_container_->pcd_container_[lastIndex]->GetFrameDescriptors().type() ==
                pcd_container_->pcd_container_[i]->GetFrameDescriptors().type()) &&
               (pcd_container_->pcd_container_[lastIndex]->GetFrameDescriptors().cols ==
                pcd_container_->pcd_container_[i]->GetFrameDescriptors().cols) ) {

            matches.clear();
            // returns the best match
            feature_matcher_.match(pcd_container_->pcd_container_[lastIndex]->GetFrameDescriptors(),
                                   pcd_container_->pcd_container_[i]->GetFrameDescriptors(), matches);

            if (matches.size() <= 200)
              continue;

            for(int k = 0; k < matches.size(); k++) {
              if (matches[k].distance <= orb_dist_th_) {
                noOfGoodMatches++;
                good_matches.push_back(matches[k]);
              }
              //LOGE("distance %f",matches[i].distance);
            }

            if (matches.size() - noOfGoodMatches <= 50) {

              NeighborWithDistance neighbor;
              neighbor.distance = translation_distance_;
              neighbor.id = i;
              neighbor.fm = true;
              neighbor.no_matches = noOfGoodMatches;
              orderedNeighbors.push_back(neighbor);
              int diff = (std::clock() - start_fm) / (double)(CLOCKS_PER_SEC / 1000);
              LOGE("Feature matcher  ------- time %i", diff);
              /*cv::Mat outputImage;
              cv::drawMatches(pcd_container_->pcd_container_[lastIndex]->GetFrame(),
                              pcd_container_->pcd_container_[lastIndex]->GetFrameKeyPoints(),
                              pcd_container_->pcd_container_[i]->GetFrame(),
                              pcd_container_->pcd_container_[i]->GetFrameKeyPoints(),
                              matches, outputImage,
                              cv::Scalar(0, 0, 255),
                              cv::Scalar(255, 0, 0));

              char filename[1024];
              sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/poss-%i-%i.jpg", lastIndex, i);
              cv::imwrite(filename, outputImage);
              LOGE("%i %i noOfGoodMatches %i size %i", i, loop_closure_frame_idx_, noOfGoodMatches, matches.size());*/
            }
          }
        }

        if (translation_distance_ <= 10) {
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance_;
          neighbor.id = i;
          orderedNeighbors.push_back(neighbor);
        }
      }
    }

    float overlap = 0;
    for (int j = 0 ; j < orderedNeighbors.size() ; j++) {
      std::sort(orderedNeighbors.begin(), orderedNeighbors.end());
      // if we already have a good matche continue. If last match has not an overlap >= 0.85 try next
      if (j > 0 && abs(orderedNeighbors[j-1].id - abs(orderedNeighbors[j].id) < 10))
        continue;

      distance_ = orderedNeighbors[j].distance;
      loop_closure_frame_idx_ = orderedNeighbors[j].id;

      //if (loop_closure_count_ % 2 == 0) {
      std::clock_t start = std::clock();
      Eigen::Isometry3f loop_pose = scan_matcher_->Match2(&overlap,
                                                         pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud(),
                                                         pcd_container_->pcd_container_[lastIndex]->GetPointCloud(),
                                                         pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPose(),
                                                         pcd_container_->pcd_container_[lastIndex]->GetPose());

      float distance_after = 100*GetDistance(glm::vec3(0,0,0), glm::vec3(loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z()));

      if(isnan(overlap))
        continue;

      if (orderedNeighbors[j].fm == true) {
        LOGE("Feature matcher overlap: %f", overlap);
        /*cv::Mat outputImage;
        cv::drawMatches(pcd_container_->pcd_container_[lastIndex]->GetFrame(),
                        pcd_container_->pcd_container_[lastIndex]->GetFrameKeyPoints(),
                        pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetFrame(),
                        pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetFrameKeyPoints(),
                        matches, outputImage,
                        cv::Scalar(0, 0, 255),
                        cv::Scalar(255, 0, 0));

        char filename[1024];
        sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/%i-%i.jpg", lastIndex, loop_closure_frame_idx_);
        cv::imwrite(filename, outputImage);
        LOGE("%i %i noOfGoodMatches %i size %i", lastIndex, loop_closure_frame_idx_, orderedNeighbors[j].no_matches, matches.size());*/
      }

      if (overlap >= 0.85 && std::fabs(distance_after - distance_) <= 5.0f) {
        LOGE("GOOD LOOP");
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
        int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
        LOGE("Scan matcher  ---------- time %i", diff);
      } else {
        LOGE("BAD LOOP");
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
        int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
        LOGE("Scan matcher  ---------- time %i", diff);
      }
    }
  }

  void LoopClosureDetector::ComputeAll(int lastIndex) {

    bool first = true;
    std::vector<NeighborWithDistance> orderedNeighbors;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    int noOfGoodMatches = 0;

    for (int current = 0; current < lastIndex; current++) {

      curr_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[current]->GetPose());

      orderedNeighbors.clear();

      for (int previous = 0; previous < current - 10; previous++) {
        prev_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[previous]->GetPose());

        rotation_distance_ = 1 - pow((curr_rotation_.w * prev_rotation_.w +
                                      curr_rotation_.x * prev_rotation_.x +
                                      curr_rotation_.y * prev_rotation_.y +
                                      curr_rotation_.z * prev_rotation_.z), 2);

        translation_distance_ = 100 * GetDistance(
            pcd_container_->pcd_container_[current]->GetTranslation(),
            pcd_container_->pcd_container_[previous]->GetTranslation());

        if (translation_distance_ <= 20 && translation_distance_ > 10) {

          std::clock_t start_fm = std::clock();

          if ((pcd_container_->pcd_container_[current]->GetFrameDescriptors().type() ==
               pcd_container_->pcd_container_[previous]->GetFrameDescriptors().type()) &&
              (pcd_container_->pcd_container_[current]->GetFrameDescriptors().cols ==
               pcd_container_->pcd_container_[previous]->GetFrameDescriptors().cols)) {

            matches.clear();
            // returns the best match
            feature_matcher_.match(pcd_container_->pcd_container_[current]->GetFrameDescriptors(),
                                   pcd_container_->pcd_container_[previous]->GetFrameDescriptors(),
                                   matches);

            if (matches.size() <= 200)
              continue;

            for (int k = 0; k < matches.size(); k++) {
              if (matches[k].distance <= orb_dist_th_) {
                noOfGoodMatches++;
                good_matches.push_back(matches[k]);
              }
              //LOGE("distance %f",matches[i].distance);
            }

            if (matches.size() - noOfGoodMatches <= 50) {

              NeighborWithDistance neighbor;
              neighbor.distance = translation_distance_;
              neighbor.id = previous;
              neighbor.fm = true;
              neighbor.no_matches = noOfGoodMatches;
              orderedNeighbors.push_back(neighbor);
              int diff = (std::clock() - start_fm) / (double) (CLOCKS_PER_SEC / 1000);
              LOGE("Feature matcher  ------- time %i", diff);
              /*cv::Mat outputImage;
              cv::drawMatches(pcd_container_->pcd_container_[lastIndex]->GetFrame(),
                              pcd_container_->pcd_container_[lastIndex]->GetFrameKeyPoints(),
                              pcd_container_->pcd_container_[i]->GetFrame(),
                              pcd_container_->pcd_container_[i]->GetFrameKeyPoints(),
                              matches, outputImage,
                              cv::Scalar(0, 0, 255),
                              cv::Scalar(255, 0, 0));

              char filename[1024];
              sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/poss-%i-%i.jpg", lastIndex, i);
              cv::imwrite(filename, outputImage);
              LOGE("%i %i noOfGoodMatches %i size %i", i, loop_closure_frame_idx_, noOfGoodMatches, matches.size());*/
            }
          }
        }

        if (translation_distance_ <= 10) {
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance_;
          neighbor.id = previous;
          orderedNeighbors.push_back(neighbor);
        }
      }

      float overlap = 0;
      for (int j = 0; j < orderedNeighbors.size(); j++) {
        std::sort(orderedNeighbors.begin(), orderedNeighbors.end());
        // check only for best frame of a round,
        if (j > 0 && abs(orderedNeighbors[j - 1].id - abs(orderedNeighbors[j].id) < 10))
          continue;

        distance_ = orderedNeighbors[j].distance;
        loop_closure_frame_idx_ = orderedNeighbors[j].id;

        //if (loop_closure_count_ % 2 == 0) {
        std::clock_t start = std::clock();
        Eigen::Isometry3f loop_pose = scan_matcher_->Match2(&overlap,
                                                            pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud(),
                                                            pcd_container_->pcd_container_[current]->GetPointCloud(),
                                                            pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPose(),
                                                            pcd_container_->pcd_container_[current]->GetPose());

        float distance_after = 100 * GetDistance(glm::vec3(0, 0, 0),
                                                 glm::vec3(loop_pose.translation().x(),
                                                           loop_pose.translation().y(),
                                                           loop_pose.translation().z()));

        if (isnan(overlap))
          continue;

        if (overlap >= 0.85 && std::fabs(distance_after - distance_) <= 5.0f) {
          LOGE("GOOD LOOP");
          Eigen::Quaternionf eigen_rot(loop_pose.rotation());
          glm::vec3 translation = glm::vec3(loop_pose.translation().x(),
                                            loop_pose.translation().y(),
                                            loop_pose.translation().z());
          glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                         eigen_rot.z());
          if (orderedNeighbors[j].fm == true) {
            LOGE("fm == true");
          }
          LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
          LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
          LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, current, orderedNeighbors[j].distance);
          LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, current, distance_after);
          LOGE("overlap: %f", overlap);
          loop_closure_poses_.insert(std::pair<key, value>(key(loop_closure_frame_idx_, current), value(distance_, loop_pose)));
          int diff = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
          LOGE("Scan matcher  ---------- time %i", diff);
        } /*else {
          LOGE("BAD LOOP");
          Eigen::Quaternionf eigen_rot(loop_pose.rotation());
          glm::vec3 translation = glm::vec3(loop_pose.translation().x(),
                                            loop_pose.translation().y(),
                                            loop_pose.translation().z());
          glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                         eigen_rot.z());
          if (orderedNeighbors[j].fm == true) {
            LOGE("fm == true");
          }
          LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
          LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
          LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, current, orderedNeighbors[j].distance);
          LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, current, distance_after);
          LOGE("overlap: %f", overlap);
          int diff = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
          LOGE("Scan matcher  ---------- time %i", diff);
        }*/
      }
    }
  }

  void LoopClosureDetector::ComputeAllPCL(int lastIndex) {
    bool first = true;
    std::vector<NeighborWithDistance> orderedNeighbors;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    int noOfGoodMatches = 0;

    for (int current = 0; current < lastIndex; current++) {

      curr_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[current]->GetPose());

      orderedNeighbors.clear();

      for (int previous = 0; previous < current - 10; previous++) {
        prev_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[previous]->GetPose());

        rotation_distance_ = 1 - pow((curr_rotation_.w * prev_rotation_.w +
                                      curr_rotation_.x * prev_rotation_.x +
                                      curr_rotation_.y * prev_rotation_.y +
                                      curr_rotation_.z * prev_rotation_.z), 2);

        translation_distance_ = 100 * GetDistance(
            pcd_container_->pcd_container_[current]->GetTranslation(),
            pcd_container_->pcd_container_[previous]->GetTranslation());

        if (translation_distance_ <= 20 && translation_distance_ > 10) {

          std::clock_t start_fm = std::clock();

          if ((pcd_container_->pcd_container_[current]->GetFrameDescriptors().type() ==
               pcd_container_->pcd_container_[previous]->GetFrameDescriptors().type()) &&
              (pcd_container_->pcd_container_[current]->GetFrameDescriptors().cols ==
               pcd_container_->pcd_container_[previous]->GetFrameDescriptors().cols)) {

            matches.clear();
            // returns the best match
            feature_matcher_.match(pcd_container_->pcd_container_[current]->GetFrameDescriptors(),
                                   pcd_container_->pcd_container_[previous]->GetFrameDescriptors(),
                                   matches);

            if (matches.size() <= 200)
              continue;

            for (int k = 0; k < matches.size(); k++) {
              if (matches[k].distance <= orb_dist_th_) {
                noOfGoodMatches++;
                good_matches.push_back(matches[k]);
              }
              //LOGE("distance %f",matches[i].distance);
            }

            if (matches.size() - noOfGoodMatches <= 50) {

              NeighborWithDistance neighbor;
              neighbor.distance = translation_distance_;
              neighbor.id = previous;
              neighbor.fm = true;
              neighbor.no_matches = noOfGoodMatches;
              orderedNeighbors.push_back(neighbor);
              int diff = (std::clock() - start_fm) / (double) (CLOCKS_PER_SEC / 1000);
              LOGE("Feature matcher  ------- time %i", diff);
              /*cv::Mat outputImage;
              cv::drawMatches(pcd_container_->pcd_container_[lastIndex]->GetFrame(),
                              pcd_container_->pcd_container_[lastIndex]->GetFrameKeyPoints(),
                              pcd_container_->pcd_container_[i]->GetFrame(),
                              pcd_container_->pcd_container_[i]->GetFrameKeyPoints(),
                              matches, outputImage,
                              cv::Scalar(0, 0, 255),
                              cv::Scalar(255, 0, 0));

              char filename[1024];
              sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/poss-%i-%i.jpg", lastIndex, i);
              cv::imwrite(filename, outputImage);
              LOGE("%i %i noOfGoodMatches %i size %i", i, loop_closure_frame_idx_, noOfGoodMatches, matches.size());*/
            }
          }
        }

        if (translation_distance_ <= 10) {
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance_;
          neighbor.id = previous;
          orderedNeighbors.push_back(neighbor);
        }
      }

      float overlap = 0;
      for (int j = 0; j < orderedNeighbors.size(); j++) {
        std::sort(orderedNeighbors.begin(), orderedNeighbors.end());
        // check only for best frame of a round,
        if (j > 0 && abs(orderedNeighbors[j - 1].id - abs(orderedNeighbors[j].id) < 10))
          continue;

        distance_ = orderedNeighbors[j].distance;
        loop_closure_frame_idx_ = orderedNeighbors[j].id;

        //if (loop_closure_count_ % 2 == 0) {
        std::clock_t start = std::clock();

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        // Defining a rotation matrix and translation vector

        //Eigen::Isometry3f odometryPose_prev = util::ConvertGLMToEigenPose(pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPose());
        //Eigen::Isometry3f odometryPose_curr = util::ConvertGLMToEigenPose(pcd_container_->pcd_container_[current]->GetPose());

        Eigen::Isometry3f odometryPose_prev(Eigen::Isometry3f(
            Eigen::Translation3f(
                pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()->sensor_origin_[0],
                pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()->sensor_origin_[1],
                pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()->sensor_origin_[2])) *
                Eigen::Isometry3f(pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()->sensor_orientation_));

        Eigen::Isometry3f odometryPose_curr(Eigen::Isometry3f(
            Eigen::Translation3f(
                pcd_container_->pcd_container_[current]->GetPointCloud()->sensor_origin_[0],
                pcd_container_->pcd_container_[current]->GetPointCloud()->sensor_origin_[1],
                pcd_container_->pcd_container_[current]->GetPointCloud()->sensor_origin_[2])) *
                Eigen::Isometry3f(pcd_container_->pcd_container_[current]->GetPointCloud()->sensor_orientation_));

        Eigen::Matrix4f initial_guess = (odometryPose_prev.inverse() * odometryPose_curr).matrix();  // prev_T_curr relative transformation
        Eigen::Quaternionf rotation;
        LOGE("inital guess: ");
        Eigen::Isometry3f inital = odometryPose_prev.inverse() * odometryPose_curr;
        LOGE("%f %f %f   %f %f %f %f", inital.translation().x(), inital.translation().y(), inital.translation().z(),
             rotation.w(), rotation.x(), rotation.y(), rotation.z());

        icp.setInputSource(pcd_container_->pcd_container_[current]->GetPointCloud());
        icp.setInputTarget(pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud());
        icp.setMaximumIterations(50);

        pcl::PointCloud<pcl::PointXYZRGB> cloud_final;
        icp.align(cloud_final, initial_guess);

        // check the overlap to evaluate the quality of the icp transformation
        pcl::RangeImage rangeImage_prev, rangeImage_curr;
        // (	const PointCloudType & 	point_cloud, float 	angular_resolution_x = pcl::deg2rad (0.5f), float 	angular_resolution_y = pcl::deg2rad (0.5f),
        // float 	max_angle_width = pcl::deg2rad (360.0f), float 	max_angle_height = pcl::deg2rad (180.0f), const Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity (),
        // RangeImage::CoordinateFrame 	coordinate_frame = CAMERA_FRAME, float 	noise_level = 0.0f, float 	min_range = 0.0f, int 	border_size = 0 )
        rangeImage_prev.createFromPointCloud(*(pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()), pcl::deg2rad(1.23f),
                                             pcl::deg2rad(0.69f),
                                             pcl::deg2rad(260.714000f),
                                             pcl::deg2rad(260.769000f),
                                             Eigen::Affine3f::Identity(),
                                             pcl::RangeImage::CAMERA_FRAME,
                                             0.0f,
                                             0.0f,
                                             0);

        rangeImage_curr.createFromPointCloud(*(pcd_container_->pcd_container_[current]->GetPointCloud()), pcl::deg2rad(1.23f),
                                             pcl::deg2rad(0.69f),
                                             pcl::deg2rad(260.714000f),
                                             pcl::deg2rad(260.769000f),
                                             Eigen::Affine3f::Identity(),
                                             pcl::RangeImage::CAMERA_FRAME,
                                             0.0f,
                                             0.0f,
                                             0);

        Eigen::Affine3f final_transform_tmp;
        final_transform_tmp.matrix() = icp.getFinalTransformation();
        float overlap = rangeImage_prev.getOverlap(rangeImage_curr, final_transform_tmp, 1, 0.02f, 2);

        Eigen::Isometry3f loop_pose;
        loop_pose.matrix() = icp.getFinalTransformation();
        rotation = loop_pose.rotation();
        LOGE("loop pose: ");
        LOGE("%f %f %f   %f %f %f %f", loop_pose.translation().x(), loop_pose.translation().y(), loop_pose.translation().z(),
             rotation.w(), rotation.x(), rotation.y(), rotation.z());

        int diff = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
        LOGE("PCL Scan matcher  ------- time %i", diff);
        LOGE("frames %i : %i dist : %f", loop_closure_frame_idx_, current, orderedNeighbors[j].distance);
        LOGE("overlap: %f", overlap);

        float distance_after = 100 * GetDistance(glm::vec3(0, 0, 0),
                                                 glm::vec3(loop_pose.translation().x(),
                                                           loop_pose.translation().y(),
                                                           loop_pose.translation().z()));

        if (icp.hasConverged() && overlap >= 0.85f && std::fabs(distance_after - distance_) <= 5.0f) {
          LOGE("GOOD LOOP");
          Eigen::Quaternionf eigen_rot(loop_pose.rotation());
          glm::vec3 translation = glm::vec3(loop_pose.translation().x(),
                                            loop_pose.translation().y(),
                                            loop_pose.translation().z());
          glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                         eigen_rot.z());
          if (orderedNeighbors[j].fm == true) {
            LOGE("fm == true");
          }
          LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
          LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
          LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, current, orderedNeighbors[j].distance);
          LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, current, distance_after);
          LOGE("overlap: %f", overlap);
          loop_closure_poses_.insert(std::pair<key, value>(key(loop_closure_frame_idx_, current), value(distance_, loop_pose)));
        } /*else {
          LOGE("BAD LOOP");
          Eigen::Quaternionf eigen_rot(loop_pose.rotation());
          glm::vec3 translation = glm::vec3(loop_pose.translation().x(),
                                            loop_pose.translation().y(),
                                            loop_pose.translation().z());
          glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                         eigen_rot.z());
          if (orderedNeighbors[j].fm == true) {
            LOGE("fm == true");
          }
          LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
          LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
          LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, current, orderedNeighbors[j].distance);
          LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, current, distance_after);
          LOGE("overlap: %f", overlap);
          int diff = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
          LOGE("Scan matcher  ---------- time %i", diff);
        }*/
      }
    }
  }


  void LoopClosureDetector::ComputeAllSM(int lastIndex) {
    bool first = true;
    std::vector<NeighborWithDistance> orderedNeighbors;
    std::vector<cv::DMatch> matches;
    std::vector<cv::DMatch> good_matches;
    int noOfGoodMatches = 0;

    for (int current = 0; current < lastIndex; current++) {

      curr_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[current]->GetPose());

      orderedNeighbors.clear();

      for (int previous = 0; previous < current - 10; previous++) {
        prev_rotation_ = glm::quat_cast(pcd_container_->pcd_container_[previous]->GetPose());

        rotation_distance_ = 1 - pow((curr_rotation_.w * prev_rotation_.w +
                                      curr_rotation_.x * prev_rotation_.x +
                                      curr_rotation_.y * prev_rotation_.y +
                                      curr_rotation_.z * prev_rotation_.z), 2);

        translation_distance_ = 100 * GetDistance(
            pcd_container_->pcd_container_[current]->GetTranslation(),
            pcd_container_->pcd_container_[previous]->GetTranslation());

        if (translation_distance_ <= 25 && translation_distance_ > 15) {

          std::clock_t start_fm = std::clock();

          if ((pcd_container_->pcd_container_[current]->GetFrameDescriptors().type() ==
               pcd_container_->pcd_container_[previous]->GetFrameDescriptors().type()) &&
              (pcd_container_->pcd_container_[current]->GetFrameDescriptors().cols ==
               pcd_container_->pcd_container_[previous]->GetFrameDescriptors().cols)) {

            matches.clear();
            good_matches.clear();
            // returns the best match
            feature_matcher_.match(pcd_container_->pcd_container_[current]->GetFrameDescriptors(),
                                   pcd_container_->pcd_container_[previous]->GetFrameDescriptors(),
                                   matches);

            if (matches.size() <= 200) {
              LOGE("matches size %i", matches.size());
              continue;
            } else {
              LOGE("matches size %i", matches.size());
            }

            for (int k = 0; k < matches.size(); k++) {
              if (matches[k].distance <= orb_dist_th_) {
                noOfGoodMatches++;
                good_matches.push_back(matches[k]);
              }
              //LOGE("distance %f",matches[i].distance);
            }

            LOGE("good matches size %i", good_matches.size());

            if (matches.size() - noOfGoodMatches <= 50) {

              NeighborWithDistance neighbor;
              neighbor.distance = translation_distance_;
              neighbor.id = previous;
              neighbor.fm = true;
              neighbor.no_matches = noOfGoodMatches;
              orderedNeighbors.push_back(neighbor);
              int diff = (std::clock() - start_fm) / (double) (CLOCKS_PER_SEC / 1000);
              LOGE("Feature matcher  ------- time %i", diff);
              /*cv::Mat outputImage;
              cv::drawMatches(pcd_container_->pcd_container_[lastIndex]->GetFrame(),
                              pcd_container_->pcd_container_[lastIndex]->GetFrameKeyPoints(),
                              pcd_container_->pcd_container_[i]->GetFrame(),
                              pcd_container_->pcd_container_[i]->GetFrameKeyPoints(),
                              matches, outputImage,
                              cv::Scalar(0, 0, 255),
                              cv::Scalar(255, 0, 0));

              char filename[1024];
              sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img/poss-%i-%i.jpg", lastIndex, i);
              cv::imwrite(filename, outputImage);
              LOGE("%i %i noOfGoodMatches %i size %i", i, loop_closure_frame_idx_, noOfGoodMatches, matches.size());*/
            }
          }
        }

        if (translation_distance_ <= 15) {
          NeighborWithDistance neighbor;
          neighbor.distance = translation_distance_;
          neighbor.id = previous;
          orderedNeighbors.push_back(neighbor);
        }
      }

      float overlap = 0;
      for (int j = 0; j < orderedNeighbors.size(); j++) {
        std::sort(orderedNeighbors.begin(), orderedNeighbors.end());
        // check only for best frame of a round,
        if (j > 0 && abs(orderedNeighbors[j - 1].id - abs(orderedNeighbors[j].id) < 10))
          continue;

        distance_ = orderedNeighbors[j].distance;
        loop_closure_frame_idx_ = orderedNeighbors[j].id;

        //if (loop_closure_count_ % 2 == 0) {
        std::clock_t start = std::clock();

        Eigen::Isometry3f loop_pose = scan_matcher_->Match2(&overlap,
                                                            pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud(),
                                                            pcd_container_->pcd_container_[current]->GetPointCloud(),
                                                            pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPose(),
                                                            pcd_container_->pcd_container_[current]->GetPose());

        if(isnan(overlap))
          continue;

        int diff = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
        LOGE("Scan matcher  ---------- time %i", diff);
        LOGE("frames %i : %i dist : %f", loop_closure_frame_idx_, current, orderedNeighbors[j].distance);
        LOGE("overlap: %f", overlap);

        float distance_after = 100 * GetDistance(glm::vec3(0, 0, 0),
                                                 glm::vec3(loop_pose.translation().x(),
                                                           loop_pose.translation().y(),
                                                           loop_pose.translation().z()));

        if (overlap >= 0.80f && std::fabs(distance_after - distance_) <= 5.0f) {
          LOGE("GOOD LOOP");
          Eigen::Quaternionf eigen_rot(loop_pose.rotation());
          glm::vec3 translation = glm::vec3(loop_pose.translation().x(),
                                            loop_pose.translation().y(),
                                            loop_pose.translation().z());
          glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                         eigen_rot.z());
          if (orderedNeighbors[j].fm == true) {
            LOGE("fm == true");
          }
          LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
          LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
          LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, current, orderedNeighbors[j].distance);
          LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, current, distance_after);
          LOGE("overlap: %f", overlap);
          loop_closure_poses_.insert(std::pair<key, value>(key(loop_closure_frame_idx_, current), value(distance_, loop_pose)));
        }

        if(overlap < 0.80 && distance_ < 10) {
          LOGE("TRY ICP \n");
          pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

          icp.setInputSource(pcd_container_->pcd_container_[current]->GetPointCloud());
          icp.setInputTarget(pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud());
          icp.setMaximumIterations(50);

          pcl::PointCloud<pcl::PointXYZRGB> cloud_final;

          Eigen::Isometry3f odometryPose_prev(Eigen::Isometry3f(
              Eigen::Translation3f(
                  pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()->sensor_origin_[0],
                  pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()->sensor_origin_[1],
                  pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()->sensor_origin_[2])) *
                                              Eigen::Isometry3f(pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()->sensor_orientation_));

          Eigen::Isometry3f odometryPose_curr(Eigen::Isometry3f(
              Eigen::Translation3f(
                  pcd_container_->pcd_container_[current]->GetPointCloud()->sensor_origin_[0],
                  pcd_container_->pcd_container_[current]->GetPointCloud()->sensor_origin_[1],
                  pcd_container_->pcd_container_[current]->GetPointCloud()->sensor_origin_[2])) *
                                              Eigen::Isometry3f(pcd_container_->pcd_container_[current]->GetPointCloud()->sensor_orientation_));

          Eigen::Matrix4f initial_guess = (odometryPose_prev.inverse() * odometryPose_curr).matrix();

          icp.align(cloud_final, initial_guess);

          // check the overlap to evaluate the quality of the icp transformation
          pcl::RangeImage rangeImage_prev, rangeImage_curr;
          // (	const PointCloudType & 	point_cloud, float 	angular_resolution_x = pcl::deg2rad (0.5f), float 	angular_resolution_y = pcl::deg2rad (0.5f),
          // float 	max_angle_width = pcl::deg2rad (360.0f), float 	max_angle_height = pcl::deg2rad (180.0f), const Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity (),
          // RangeImage::CoordinateFrame 	coordinate_frame = CAMERA_FRAME, float 	noise_level = 0.0f, float 	min_range = 0.0f, int 	border_size = 0 )
          rangeImage_prev.createFromPointCloud(*(pcd_container_->pcd_container_[loop_closure_frame_idx_]->GetPointCloud()), pcl::deg2rad(1.23f),
                                               pcl::deg2rad(0.69f),
                                               pcl::deg2rad(260.714000f),
                                               pcl::deg2rad(260.769000f),
                                               Eigen::Affine3f::Identity(),
                                               pcl::RangeImage::CAMERA_FRAME,
                                               0.0f,
                                               0.0f,
                                               0);

          rangeImage_curr.createFromPointCloud(*(pcd_container_->pcd_container_[current]->GetPointCloud()), pcl::deg2rad(1.23f),
                                               pcl::deg2rad(0.69f),
                                               pcl::deg2rad(260.714000f),
                                               pcl::deg2rad(260.769000f),
                                               Eigen::Affine3f::Identity(),
                                               pcl::RangeImage::CAMERA_FRAME,
                                               0.0f,
                                               0.0f,
                                               0);

          Eigen::Affine3f final_transform_tmp;
          final_transform_tmp.matrix() = icp.getFinalTransformation();
          float overlap_icp = rangeImage_prev.getOverlap(rangeImage_curr, final_transform_tmp, 1, 0.02f, 2);

          loop_pose.matrix() = icp.getFinalTransformation();
          LOGE("ICP overlap: %f", overlap_icp);
          if (overlap_icp > overlap && overlap_icp > 70) {
            LOGE("ICP GOOD");
            loop_closure_poses_.insert(std::pair<key, value>(key(loop_closure_frame_idx_, current), value(distance_, loop_pose)));
          }
        }

        /*else {
          LOGE("BAD LOOP");
          Eigen::Quaternionf eigen_rot(loop_pose.rotation());
          glm::vec3 translation = glm::vec3(loop_pose.translation().x(),
                                            loop_pose.translation().y(),
                                            loop_pose.translation().z());
          glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                         eigen_rot.z());
          if (orderedNeighbors[j].fm == true) {
            LOGE("fm == true");
          }
          LOGE("translation : %f %f %f", translation.x, translation.y, translation.z);
          LOGE("rotation : %f %f %f %f", rotation.w, rotation.x, rotation.y, rotation.z);
          LOGE("[before] %i : %i dist : %f", loop_closure_frame_idx_, current, orderedNeighbors[j].distance);
          LOGE("[after] %i : %i dist: %f", loop_closure_frame_idx_, current, distance_after);
          LOGE("overlap: %f", overlap);
          int diff = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
          LOGE("Scan matcher  ---------- time %i", diff);
        }*/
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