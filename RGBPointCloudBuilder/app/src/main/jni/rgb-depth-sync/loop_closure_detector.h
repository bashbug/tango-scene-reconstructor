#ifndef RGBPOINTCLOUDBUILDER_LOOPCLOSUREDETECTOR_H
#define RGBPOINTCLOUDBUILDER_LOOPCLOSUREDETECTOR_H

#include <map>
#include <utility>
#include <future>
#include <memory>
#include <Eigen/Geometry>
#include <tango_client_api.h>
#include <tango-gl/util.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp> // ORB and BFMatcher
#include "rgb-depth-sync/pcd_container.h"
#include "rgb-depth-sync/img_file_writer.h"
#include "rgb-depth-sync/scan_matcher.h"

namespace rgb_depth_sync {

  typedef std::pair<int, int> key;
  typedef std::pair<int, std::future<Eigen::Isometry3f> > value;

  class LoopClosureDetector {
    public:
      LoopClosureDetector(PCDContainer* pcd_container);
      ~LoopClosureDetector();
      void Compute(int lastIndex);
      void GetLoopClosurePoses(std::map<key, value>** loop_closure_poses);

    private:
      float GetDistance(const glm::vec3 &curr_trans, const glm::vec3 &prev_trans);
      float GetPoseDistance(const glm::vec3& curr_translation, const glm::quat& curr_rotation, const glm::vec3& prev_translation, const glm::quat& prev_rotation);
      float rotation_distance_;
      float translation_distance_;
      int max_loop_closure_per_frame_;
      int counter_loop_closure_per_frame_;
      cv::Ptr<cv::BFMatcher> bf_matcher_;
      cv::Ptr<cv::FlannBasedMatcher> flann_matcher_;
      std::vector<cv::DMatch> match_;
      glm::vec4 ConvertDepthPointTo3DPoint(const glm::vec3 &depth_point);
      glm::vec3 curr_translation_;
      glm::quat curr_rotation_;
      glm::vec3 prev_translation_;
      glm::quat prev_rotation_;
      cv::Ptr<cv::ORB> orb_;
      PCDContainer* pcd_container_;
      IMGFileWriter* img_file_writer_;
      std::map<key, value> loop_closure_poses_;
      std::map<key, value>::iterator loop_closure_poses_it_;
      glm::vec3 curr_depth_point_lu_;
      glm::vec4 curr_pcd_point_lu_;
      glm::vec3 curr_depth_point_ld_;
      glm::vec4 curr_pcd_point_ld_;
      glm::vec3 curr_depth_point_cc_;
      glm::vec4 curr_pcd_point_cc_;
      glm::vec3 curr_depth_point_ru_;
      glm::vec4 curr_pcd_point_ru_;
      glm::vec3 curr_depth_point_rd_;
      glm::vec4 curr_pcd_point_rd_;

      glm::vec4 transformed_pcd_point_lu_;
      glm::vec4 transformed_pcd_point_ld_;
      glm::vec4 transformed_pcd_point_cc_;
      glm::vec4 transformed_pcd_point_ru_;
      glm::vec4 transformed_pcd_point_rd_;

      float distance_;
      float distance_accum_;
      float distance_average_;
      int distance_accum_counter_;

      glm::mat4 curr_pose_;
      glm::mat4 relative_transformation_;
      TangoCameraIntrinsics color_camera_intrinsics_;
  };
}


#endif //RGBPOINTCLOUDBUILDER_LOOPCLOSUREDETECTOR_H
