/*
 * PCDOutlierRemoval class removes outliers with pcl::StatisticalOutlierRemoval
 * with a given radius. Points with a z-value which lies outside of the given
 * min and max range are removed as well.
 */

#include <vector>
#include <iostream>
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
#include <tango-gl/util.h>

#ifndef RGBPOINTCLOUDBUILDER_PCD_OUTLIER_REMOVAL_H
#define RGBPOINTCLOUDBUILDER_PCD_OUTLIER_REMOVAL_H

namespace rgb_depth_sync {

  class PCDOutlierRemoval {

    public:
      PCDOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float radius);
      ~PCDOutlierRemoval();
  };

}

#endif //RGBPOINTCLOUDBUILDER_PCD_OUTLIER_REMOVAL_H
