/*
 * Class writes rgb point cloud either to an file or
 * sends it via TCP socket.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H

#include <vector>
#include <tango-gl/util.h>
#include <tango_client_api.h>
#include <pcl/point_types.h>
#include "pcl/point_cloud.h"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "rgb-depth-sync/pose_data.h"
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  class PCD {
  public:
    PCD();
    ~PCD();
    void MapXYZWithRGB(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >& xyz,
                       const std::vector<uint8_t>& rgb,
                       double xyz_timestamp,
                       double rgb_timestamp);
    std::vector<float> GetXYZValues();
    std::vector<uint8_t> GetRGBValues();
    glm::mat4 GetPose();
    glm::vec3 GetTranslation();
    glm::quat GetRotation();
    void SetTranslation(const glm::vec3& translation);
    void SetRotation(const glm::quat rotation);
    void SetKeyPointsAndDescriptors(const std::vector<cv::KeyPoint>& frame_key_points, cv::Mat frame_descriptors);
    void SetFrame(const cv::Mat& frame);
    std::vector<float> GetXYZValuesTSS();
    std::vector<float> GetPCD();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPointCloud();
    std::vector<cv::KeyPoint> GetFrameKeyPoints();
    cv::Mat GetFrameDescriptors();
    cv::Mat GetFrame();
  private:
    std::vector<float> xyz_values_color_camera_;
    std::vector<float> xyz_values_ss_;
    std::vector<uint8_t> rgb_values_;
    std::vector<float> pcd_;
    double xyz_timestamp_;
    double rgb_timestamp_;
    glm::mat4 pose_;
    glm::quat rotation_;
    glm::vec3 translation_;
    Eigen::Matrix4f transformation_matrix_;
    std::vector<cv::KeyPoint> frame_key_points_;
    cv::Mat frame_descriptors_;
    cv::Mat frame_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  };
} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
