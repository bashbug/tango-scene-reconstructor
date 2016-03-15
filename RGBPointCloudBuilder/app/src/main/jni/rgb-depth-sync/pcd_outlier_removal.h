//
// Created by anastasia on 15.02.16.
//

#include <vector>
#include <tango-gl/util.h>

#include <boost/thread.hpp>
#include <flann/flann.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/search/impl/organized.hpp>  // to get rid of undefined reference to pcl::getCameraMatrixFromProjectionMatrix
//#include <pcl/search/impl/search.hpp>  // to get rid of undefined reference to pcl::getCameraMatrixFromProjectionMatrix
#include <pcl/filters/impl/statistical_outlier_removal.hpp>  // to get rid of undefined reference to pcl::KdTreeFLANN
#include <pcl/filters/statistical_outlier_removal.h>

/*#include <boost/filesystem.hpp>
#include <pcl/common/common.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/kdtree.h>
#include <boost/make_shared.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/ros/conversions.h>
#include <pcl/search/organized.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>

#include <pcl/console/print.h>*/
/*#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/impl/io.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>*/

#ifndef RGBPOINTCLOUDBUILDER_PCD_OUTLIER_REMOVAL_H
#define RGBPOINTCLOUDBUILDER_PCD_OUTLIER_REMOVAL_H

namespace rgb_depth_sync {

  class PCDOutlierRemoval {

    public:
      PCDOutlierRemoval();
      ~PCDOutlierRemoval();
      std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > Compute(const std::vector<float>& all_points, float min_range, float max_range, float radius);
  };

}


#endif //RGBPOINTCLOUDBUILDER_PCD_OUTLIER_REMOVAL_H
