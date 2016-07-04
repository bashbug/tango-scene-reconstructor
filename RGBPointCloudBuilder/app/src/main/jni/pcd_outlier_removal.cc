#include "rgb-depth-sync/pcd_outlier_removal.h"

namespace rgb_depth_sync {

  PCDOutlierRemoval::PCDOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (cloud->points.size() == 0)
      return;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(radius);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    cloud->height = 1;
    cloud->width = cloud_filtered->points.size();
    cloud->points = cloud_filtered->points;
  }

  PCDOutlierRemoval::~PCDOutlierRemoval() { }
}
