#ifndef RGBPOINTCLOUDBUILDER_MESH_H
#define RGBPOINTCLOUDBUILDER_MESH_H

#include <mutex>
#include <condition_variable>
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

  typedef std::map<int , Voxel> z_coord;
  typedef std::map<int, z_coord> y_coord;
  typedef std::map<int, y_coord> x_coord;

  typedef z_coord::iterator z_coord_iter;
  typedef y_coord::iterator y_coord_iter;
  typedef x_coord::iterator x_coord_iter;

  class Mesh {
    public:
      Mesh();
      ~Mesh();
      std::vector<float> GetXYZValues(glm::mat4 curr_pose);
      std::vector<uint8_t> GetRGBValues();
      void AddPointCloud(PCD* pcd);
      void AddPointCloudOptWithSM(PCD* pcd);
      void AddPointCloudOptWithMSM(PCD* pcd);
      void DownsampleMesh();
      bool Reset();
      bool IsRunning();
    private:
      int Hesh(float x_f, float y_f, float z_f);
      std::vector<float> xyz_values_;
      std::vector<uint8_t> rgb_values_;
      std::map<int, Voxel> point_cloud_;
      std::map<int, std::map<int, std::map<int, Voxel> > > map_;

      bool first_;
      int p1, p2, p3;
      int hash_table_size_;
      bool is_running_;
      int resolution_;
      std::mutex mesh_mtx_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_mesh_;
  };
}

#endif //RGBPOINTCLOUDBUILDER_MESH_H
