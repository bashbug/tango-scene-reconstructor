#include "rgb-depth-sync/multiframe_scan_matcher.h"

namespace rgb_depth_sync {

  MultiframeScanMatcher::MultiframeScanMatcher() {
    maxCorrespondenceDistance_ = 0.05;
    maxRange_ = 80.0f;
    maxAngle_ = 30.0f;
    threads_ = 1;
    g2oIterations_ = 2;
    iterations_ = 5;

    width_ = 320;
    height_ = 180;
    fx_ = 260.714000f;
    fy_ = 260.769000f;
    cx_ = 161.307000;
    cy_ = 88.500700;
  }

  MultiframeScanMatcher::~MultiframeScanMatcher() {}

  void MultiframeScanMatcher::Init(PCDContainer* pcd_container) {
    average_computation_time_ = 0;
    computation_time_ = 0;
    pcd_container_ = pcd_container;

    // compute normales for each cloud
    LOGE("Compute Normales start...");
    std::clock_t start = std::clock();

    for (int i = 0; i <= pcd_container_->GetPCDContainerLastIndex(); i++ ) {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(
          new pcl::PointCloud <pcl::PointXYZRGBNormal>);
      pcl::copyPointCloud(*pcd_container_->pcd_container_[i]->GetPointCloud(), *cloud_with_normals);

      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
      pcd_container_->pcd_container_[i]->GetPointCloud()->sensor_origin_.setZero();
      pcd_container_->pcd_container_[i]->GetPointCloud()->sensor_orientation_ = Eigen::Quaternionf::Identity();
      ne.setInputCloud (pcd_container_->pcd_container_[i]->GetPointCloud());
      ne.setKSearch (100);
      ne.compute (*cloud_with_normals);

      // downsample and organize each cloud
      std::map<int, pcl::PointXYZRGBNormal> sorted_points;
      for (int k = 0; k < cloud_with_normals->points.size(); k++) {
        int pixel_x = static_cast<int>(cloud_with_normals->points[k].x / cloud_with_normals->points[k].z * fx_ + cx_);
        int pixel_y = static_cast<int>(cloud_with_normals->points[k].y / cloud_with_normals->points[k].z * fy_ + cy_);

        int index = pixel_x + pixel_y * width_;

        if (index >= width_ * height_)
          continue;

        sorted_points[index] = cloud_with_normals->points[k];
      }

      std::map<int, pcl::PointXYZRGBNormal>::iterator it_sorted_points = sorted_points.begin();

      cloud_with_normals->points.clear();

      for (int k = 0; k < sorted_points.size(); k+=10) {
        std::advance(it_sorted_points, 10);
        cloud_with_normals->points.push_back(it_sorted_points->second);
      }

      cloud_with_normals->height = 1;
      cloud_with_normals->width = cloud_with_normals->points.size();
      clouds_.push_back(cloud_with_normals);
    }

    int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);

    LOGE("Compute Normales stops after %i ms", diff);
  }

  void MultiframeScanMatcher::Optimize() {
    LOGE("Multiframe ICP init start...");
    MultiFrameIcp multiFrameIcp;
    multiFrameIcp.params.maxCorrespondenceDistance = maxCorrespondenceDistance_;
    multiFrameIcp.params.maxRange = maxRange_;
    multiFrameIcp.params.maxAngle = util::Deg2Rad(maxAngle_);
    multiFrameIcp.params.threads = threads_;
    multiFrameIcp.params.iterations = g2oIterations_;

    std::clock_t start = std::clock();

    LOGE("Multiframe ICP init graph start...");
    multiFrameIcp.initGraph(clouds_);
    LOGE("Multiframe ICP init graph stop...");

    int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
    computation_time_ = diff;

    LOGE("Multiframe ICP init stops after %i ms", diff);

    LOGE("Multiframe ICP optimize start...");
    start = std::clock();

    for (int iteration=0; iteration<iterations_; ++iteration)  {
      // Do outer loop iterations
      multiFrameIcp.optimize(clouds_);
    }

    diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
    computation_time_ += diff;

    average_computation_time_ = diff/iterations_;

    LOGE("Multiframe ICP optimize stops after %i ms", diff);

    for(int i = 0; i < (int) clouds_.size(); i++) {
      Eigen::Isometry3f pose;
      multiFrameIcp.getPose(i, pose);
      Eigen::Quaternionf eigen_rot(pose.rotation());
      glm::vec3 translation = glm::vec3(pose.translation().x(),
                                        pose.translation().y(),
                                        pose.translation().z());
      glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                     eigen_rot.z());
      pcd_container_->pcd_container_[i]->SetMFSMPose(pose);
      pcd_container_->pcd_container_[i]->SetTranslationMFSM(translation);
      pcd_container_->pcd_container_[i]->SetRotationMFSM(rotation);
    }
  }

  int MultiframeScanMatcher::GetAverageComputationTime() {
    return average_computation_time_;
  }

  int MultiframeScanMatcher::GetComputationTime() {
    return computation_time_;
  }
}
