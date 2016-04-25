/**
 * Multi-Frame Scan Matcher:
 *
 * Post-processing method to optimize poses with range data.
 * Graph-based 3D SLAM but refined as a joint optimization task which simultaneously
 * estimates the device poses and the optimal position of the points of each point cloud.
 * see ruhnke2011range, ruhnke2011highly
 *
 * Uses g2o as graph optimization system with Levenberg-Marquardt and a CSparse solver.
 * see grisetti2010tutorial, grisetti2011g2o, kummerle2011g
 */

#ifndef RGBPOINTCLOUDBUILDER_MULTIFRAME_SCAN_MATCHER_H
#define RGBPOINTCLOUDBUILDER_MULTIFRAME_SCAN_MATCHER_H

#include <iterator>     // std::advance
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/organized.hpp>  // to get rid of undefined reference to pcl::getCameraMatrixFromProjectionMatrix
#include <pcl/features/impl/normal_3d.hpp> // to get rid of undefined reference to pcl::search::OrganizedNeighbor<pcl::PointXYZI>::estimateProjectionMatrix()
#include <pcl/features/normal_3d.h>

#include "multiFrameIcp.h"
#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  class MultiframeScanMatcher {

    public:
      MultiframeScanMatcher();
      ~MultiframeScanMatcher();
      void Init(PCDContainer* pcd_container);
      void Optimize();
      int GetAverageComputationTime();
      int GetComputationTime();
    private:
      std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > clouds_;
      PCDContainer* pcd_container_;
      int last_index_, threads_, g2oIterations_, iterations_;
      float maxCorrespondenceDistance_, maxRange_, maxAngle_;
      int average_computation_time_, computation_time_;
      int width_, height_;
      float fx_, fy_, cx_, cy_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_MULTIFRAME_SCAN_MATCHER_H
