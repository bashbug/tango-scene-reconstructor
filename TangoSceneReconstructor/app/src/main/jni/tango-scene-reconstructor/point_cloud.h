/**
 * PCD class calculates and holds the xyz and rgb values of a point cloud.
 * The pose of tango vio and the FTFSM and MFSM optimization will be set
 * and are accessible by mesh.h/.cc to create a merged mesh of all rgb
 * point clouds.
 */

#ifndef TANGOSCENERECONSTRUCTOR_POINT_CLOUD_H
#define TANGOSCENERECONSTRUCTOR_POINT_CLOUD_H

#include <vector>
#include <boost/thread.hpp>
#include <flann/flann.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/impl/organized.hpp>  // to get rid of undefined reference to pcl::getCameraMatrixFromProjectionMatrix
#include <pcl/filters/impl/statistical_outlier_removal.hpp>  // to get rid of undefined reference to pcl::KdTreeFLANN
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tango_client_api.h>
#include <tango_support_api.h>
#include <tango-gl/util.h>

#include "tango-scene-reconstructor/pose_data.h"
#include "tango-scene-reconstructor/util.h"

namespace tango_scene_reconstructor {

  class PointCloud {

    public:
      PointCloud(float far_clipping=2.5f);
      ~PointCloud();
      void SetXYZ(TangoXYZij* XYZij);
      void SetYUV(TangoImageBuffer* YUV);
      TangoXYZij* GetXYZ();
      TangoImageBuffer* GetYUV();
      void SetNearClipping(float near_clipping);
      void SetFarClipping(float far_clipping);
      void SetTranslation(const glm::vec3& translation);
      void SetRotation(const glm::quat& rotation);
      void SetTranslationFTFSM(const glm::vec3& translation);
      void SetRotationFTFSM(const glm::quat& rotation);
      void SetTranslationMFSM(const glm::vec3& translation);
      void SetRotationMFSM(const glm::quat& rotation);
      void SaveRGBImage(const char* path, int id);
      void SetFTFSMPose(Eigen::Isometry3f ftfsm_pose);
      void SetMFSMPose(Eigen::Isometry3f mfsm_pose);

      void Update();
      void RemoveOutliers(float radius);
      void SaveAsPCD(const char* filename);
      void SaveAsPCDWithFTFSMPose(const char* filename);
      void SaveAsPCDWithMFSMPose(const char* filename);

      std::vector<float> GetXYZValues();
      std::vector<uint8_t> GetRGBValues();
      glm::mat4 GetPose();
      glm::vec3 GetTranslation();
      glm::quat GetRotation();
      Eigen::Isometry3f GetFTFSMPose();
      glm::vec3 GetTranslationFTFSM();
      glm::quat GetRotationFTFSM();
      Eigen::Isometry3f GetMFSMPose();
      glm::vec3 GetTranslationMFSM();
      glm::quat GetRotationMFSM();
      std::vector<float> GetXYZValuesTSS();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPointCloud();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPointCloudTransformed();

      glm::mat4 yuv_pose_;
      glm::mat4 xyz_pose_;

    private:
      void SetRGB();
      PoseData* pose_data_;

      std::vector<float> xyz_values_color_camera_;
      std::vector<float> xyz_values_ss_;
      std::vector<uint8_t> rgb_values_;

      double xyz_timestamp_;
      double rgb_timestamp_;

      glm::mat4 pose_;
      glm::quat rotation_;
      glm::vec3 translation_;
      glm::quat rotation_ftfsm_;
      glm::vec3 translation_ftfsm_;
      glm::quat rotation_mfsm_;
      glm::vec3 translation_mfsm_;
      Eigen::Isometry3f ftfsm_pose_;
      Eigen::Isometry3f mfsm_pose_;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed_;

      TangoXYZij XYZij_;
      TangoImageBuffer YUV_;

      uint32_t yuv_size_;
      uint32_t rgb_size_;
      cv::Mat yuv_frame_;
      cv::Mat rgb_frame_;

      float near_clipping_, far_clipping_;
  };

} // namespace rgb_depth_sync

#endif // TANGOSCENERECONSTRUCTOR_POINT_CLOUD_H
