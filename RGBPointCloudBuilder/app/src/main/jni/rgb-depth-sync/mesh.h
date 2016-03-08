#ifndef RGBPOINTCLOUDBUILDER_MESH_H
#define RGBPOINTCLOUDBUILDER_MESH_H

#include <boost/make_shared.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread.hpp>
#include <flann/flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/kdtree.h>
#include <boost/make_shared.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/ros/conversions.h>
#include <pcl/search/organized.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>

#include <pcl/console/print.h>

#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/pcd.h"

namespace rgb_depth_sync {

  struct Voxel {
    float x, y, z;
    uint8_t r, g, b;
  };

  class Mesh {
    public:
      Mesh();
      ~Mesh();
      std::vector<float> GetXYZValues(glm::mat4 curr_pose);
      std::vector<uint8_t> GetRGBValues();
      void AddPointCloud(PCD* pcd);

    private:
      int Hesh(float x_f, float y_f, float z_f);
      std::vector<float> xyz_values_;
      std::vector<uint8_t> rgb_values_;
      std::map<int, Voxel> point_cloud_;
      bool first_;
      int p1, p2, p3;
      int hash_table_size_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_MESH_H
