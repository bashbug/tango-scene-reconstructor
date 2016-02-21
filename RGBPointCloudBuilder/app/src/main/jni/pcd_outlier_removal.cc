//
// Created by anastasia on 15.02.16.
//
#include "rgb-depth-sync/pcd_outlier_removal.h"

namespace rgb_depth_sync {

  struct PointWithDistance {
    float distance;
  };

  PCDOutlierRemoval::PCDOutlierRemoval() {
  }

  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > PCDOutlierRemoval::Compute(const std::vector<float>& all_points, float min_range, float max_range, float radius) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // copy xyz float array to cv matrix of Point3f values
    for(int i = 0; i < all_points.size(); i+=3) {
      if(all_points[i+2] > max_range || all_points[i+2] < min_range) {
        continue;
      }
      // remove nan points
      if (std::abs(all_points[i]) < 1e-4 &&
          std::abs(all_points[i + 1]) < 1e-4 &&
          std::abs(all_points[i + 2]) < 1e-4) {
        continue;
      }
      pcl::PointXYZ p;
      p.x = all_points[i];
      p.y = all_points[i+1];
      p.z = all_points[i+2];
      cloud->points.push_back(p);
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(5);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    return cloud_filtered->points;
  }
}
