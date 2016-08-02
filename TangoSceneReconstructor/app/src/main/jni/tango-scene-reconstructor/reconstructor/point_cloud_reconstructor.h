/*
 * Reconstruction creates the RGB point cloud object model while scanning
 * process with Tango VIO poses. It uses this poses to transform the
 * RGB point clouds to the origin.
 * A voxel grid is used to downsample the RGB point cloud object model
 * and gets rid of duplicates.
 * This class also creates the RGB point cloud object models for FTFSM
 * and MFSM. After merging, these point clouds will be filtered to
 * remove outliers.
 */

#ifndef TANGOSCENERECONSTRUCTOR_POINT_CLOUD_RECONSTRUCTOR_H
#define TANGOSCENERECONSTRUCTOR_POINT_CLOUD_RECONSTRUCTOR_H

#include <mutex>
#include <condition_variable>
#include <iostream>
#include <boost/make_shared.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <flann/flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/ros/conversions.h>
#include <pcl/search/organized.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/console/print.h>

#include "tango-scene-reconstructor/util.h"
#include "tango-scene-reconstructor/point_cloud.h"

namespace tango_scene_reconstructor {

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

  class PointCloudReconstructor {
    public:
      PointCloudReconstructor();
      ~PointCloudReconstructor();
      std::vector<float> GetXYZValues(glm::mat4 curr_pose);
      std::vector<uint8_t> GetRGBValues();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetPCDFile();
      void AddPointCloud(PointCloud* pcd);
      void AddPointCloudOptWithSM(PointCloud* pcd);
      void AddPointCloudOptWithMSM(PointCloud* pcd);
      void DownsampleMesh();
      void FilterMesh(float radius=10);
      bool Reset();
      bool IsRunning();
      glm::mat4 GetCentroidMatrix();
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
      glm::vec4 centroid_;
  };

} // namespace tango_scene_reconstructor

#endif // TANGOSCENERECONSTRUCTOR_POINT_CLOUD_RECONSTRUCTOR_H
