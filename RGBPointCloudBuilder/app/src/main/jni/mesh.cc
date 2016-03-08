//
// Created by anastasia on 04.03.16.
//
#include "rgb-depth-sync/mesh.h"

namespace rgb_depth_sync {

  Mesh::Mesh() {
    first_ = true;
    p1 = 73856093;
    p2 = 19349669;
    p3 = 83492791;
    hash_table_size_ = 400000;
  }

  Mesh::~Mesh() {

  }

  std::vector<float> Mesh::GetXYZValues(glm::mat4 curr_pose) {
    std::vector<float> xyz_values_transformed;

    rgb_values_.clear();

    for (auto const& v : point_cloud_) {
      glm::vec3 device_point = glm::vec3(curr_pose * glm::vec4 (v.second.x, v.second.y, v.second.z, 1.0f));
      xyz_values_transformed.push_back(device_point.x);
      xyz_values_transformed.push_back(device_point.y);
      xyz_values_transformed.push_back(device_point.z);

      rgb_values_.push_back(v.second.r);
      rgb_values_.push_back(v.second.g);
      rgb_values_.push_back(v.second.b);
    }

    return xyz_values_transformed;
  }

  std::vector<uint8_t> Mesh::GetRGBValues() {
    return rgb_values_;
  }

  void Mesh::AddPointCloud(PCD* pcd) {

    /*if (first_) {
      *mesh_ = *point_cloud;
      first_ = false;
    } else {
      *mesh_ += *point_cloud;
    }*/

    /*pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(mesh_);
    sor.setLeafSize(0.25f, 0.25f, 0.25f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter (*filtered);
    mesh_ = filtered;*/

    //xyz_values_.clear();
    //rgb_values_.clear();

    std::vector<float> tmp_xyz = pcd->GetXYZValuesTSS();
    std::vector<uint8_t> tmp_rgb = pcd->GetRGBValues();

    for (int i = 0; i < tmp_xyz.size(); i+=3) {
      Voxel v;
      v.x = tmp_xyz[i];
      v.y = tmp_xyz[i+1];
      v.z = tmp_xyz[i+2];
      v.r = tmp_rgb[i];
      v.g = tmp_rgb[i+1];
      v.b = tmp_rgb[i+2];
      int hesh = Hesh(v.x, v.y, v.z);
      point_cloud_[hesh] = v;
    }

    /*if (first_) {
      xyz_values_.resize(tmp_xyz.size());
      rgb_values_.resize(tmp_rgb.size());
      std::copy(tmp_xyz.begin(), tmp_xyz.end(), xyz_values_.begin());
      std::copy(tmp_rgb.begin(), tmp_rgb.end(), rgb_values_.begin());
      first_ = false;
    } else {
      xyz_values_.insert(xyz_values_.end(), tmp_xyz.begin(), tmp_xyz.end());
      rgb_values_.insert(rgb_values_.end(), tmp_rgb.begin(), tmp_rgb.end());
    }*/

    /*for (int i = 0; i < mesh_->points.size(); i++) {
      xyz_values_.push_back(mesh_->points[i].x);
      xyz_values_.push_back(mesh_->points[i].y);
      xyz_values_.push_back(mesh_->points[i].z);
      rgb_values_.push_back(mesh_->points[i].r);
      rgb_values_.push_back(mesh_->points[i].g);
      rgb_values_.push_back(mesh_->points[i].b);
    }*/
  }

  int Mesh::Hesh(float x_f, float y_f, float z_f) {
    int x = *reinterpret_cast<int*>(&x_f);
    int y = *reinterpret_cast<int*>(&y_f);
    int z = *reinterpret_cast<int*>(&z_f);
    return (x * p1 xor y * p2 xor z * p3) % hash_table_size_;

  }
}