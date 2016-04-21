//
// Created by anastasia on 04.03.16.
//
#include "rgb-depth-sync/mesh.h"

namespace rgb_depth_sync {

  Mesh::Mesh() {
    first_ = true;
    is_running_ = false;
    pcd_mesh_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  }

  Mesh::~Mesh() {

  }

  bool Mesh::IsRunning() {
    return is_running_;
  }

  bool Mesh::Reset() {
    bool r = false;

    {
      std::lock_guard<std::mutex> lock(mesh_mtx_);
      point_cloud_.clear();
      rgb_values_.clear();
      xyz_values_.clear();
      pcd_mesh_->points.clear();
      pcd_mesh_->width = 0;
      LOGE("reset done");
      r = true;
    }

    return r;
  }

  std::vector<float> Mesh::GetXYZValues(glm::mat4 curr_pose) {
    std::vector<float> xyz_values_transformed;
    rgb_values_.clear();

    {
      std::lock_guard<std::mutex> lock(mesh_mtx_);
      is_running_ = true;
      for (int i = 0; i < pcd_mesh_->points.size(); i++) {
        glm::vec3 device_point = glm::vec3(curr_pose * glm::vec4 (pcd_mesh_->points[i].x, pcd_mesh_->points[i].y, pcd_mesh_->points[i].z, 1.0f));
        xyz_values_transformed.push_back(device_point.x);
        xyz_values_transformed.push_back(device_point.y);
        xyz_values_transformed.push_back(device_point.z);
        rgb_values_.push_back(pcd_mesh_->points[i].r);
        rgb_values_.push_back(pcd_mesh_->points[i].g);
        rgb_values_.push_back(pcd_mesh_->points[i].b);
      }

      is_running_ = false;
    }

    return xyz_values_transformed;
  }

  std::vector<uint8_t> Mesh::GetRGBValues() {
    return rgb_values_;
  }

  void Mesh::AddPointCloudOptWithSM(PCD* pcd) {
    {
      std::lock_guard <std::mutex> lock(mesh_mtx_);
      is_running_ = true;

      // transfrom pcd with sm pose
      Eigen::Matrix4f transform = pcd->GetSMPose().matrix();
      // add new point cloud to mesh
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*(pcd->GetPointCloud()), *transformed_cloud, transform);

      *pcd_mesh_ += *transformed_cloud;

      is_running_ = false;
    }
  }

  void Mesh::AddPointCloudOptWithMSM(PCD* pcd) {
    {
      std::lock_guard <std::mutex> lock(mesh_mtx_);
      is_running_ = true;

      // transfrom pcd with sm pose
      Eigen::Matrix4f transform = pcd->GetMSMPose().matrix();
      // add new point cloud to mesh
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*(pcd->GetPointCloud()), *transformed_cloud, transform);

      *pcd_mesh_ += *transformed_cloud;

      is_running_ = false;
    }
  }

  void Mesh::DownsampleMesh() {
    {
      std::lock_guard <std::mutex> lock(mesh_mtx_);
      is_running_ = true;

      pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;
      // Create the filtering object
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud (pcd_mesh_);
      vg.setLeafSize (0.004f, 0.004f, 0.004f);
      vg.filter (cloud_downsampled);

      *pcd_mesh_ = cloud_downsampled;

      is_running_ = false;
    }
  }

  void Mesh::AddPointCloud(PCD* pcd) {
    {
      std::lock_guard <std::mutex> lock(mesh_mtx_);
      is_running_ = true;

      // add new point cloud to mesh
      *pcd_mesh_ += *(pcd->GetPointCloudTransformed());

      pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;

      // Create the filtering object
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud (pcd_mesh_);
      vg.setLeafSize (0.004f, 0.004f, 0.004f);
      vg.filter (cloud_downsampled);

      *pcd_mesh_ = cloud_downsampled;

      is_running_ = false;
    }
  }
}