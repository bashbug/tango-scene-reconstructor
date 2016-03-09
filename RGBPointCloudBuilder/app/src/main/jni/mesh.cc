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
    is_running_ = false;
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
      if(point_cloud_.size() > 0) {
        for (auto const& v : point_cloud_) {
          glm::vec3 device_point = glm::vec3(curr_pose * glm::vec4 (v.second.x, v.second.y, v.second.z, 1.0f));
          xyz_values_transformed.push_back(device_point.x);
          xyz_values_transformed.push_back(device_point.y);
          xyz_values_transformed.push_back(device_point.z);

          rgb_values_.push_back(v.second.r);
          rgb_values_.push_back(v.second.g);
          rgb_values_.push_back(v.second.b);
        }
      }
      is_running_ = false;
    }

    return xyz_values_transformed;
  }

  std::vector<uint8_t> Mesh::GetRGBValues() {
    return rgb_values_;
  }

  void Mesh::AddPointCloud(PCD* pcd) {
    {
      std::lock_guard <std::mutex> lock(mesh_mtx_);
      is_running_ = true;

      std::vector <float> tmp_xyz = pcd->GetXYZValuesTSS();
      std::vector <uint8_t> tmp_rgb = pcd->GetRGBValues();

      for (int i = 0; i < tmp_xyz.size(); i += 3) {
        Voxel v;
        v.x = tmp_xyz[i];
        v.y = tmp_xyz[i + 1];
        v.z = tmp_xyz[i + 2];
        v.r = tmp_rgb[i];
        v.g = tmp_rgb[i + 1];
        v.b = tmp_rgb[i + 2];
        int hesh = Hesh(v.x, v.y, v.z);
        point_cloud_[hesh] = v;
      }

      is_running_ = false;
    }
  }

  int Mesh::Hesh(float x_f, float y_f, float z_f) {
    int x = *reinterpret_cast<int*>(&x_f);
    int y = *reinterpret_cast<int*>(&y_f);
    int z = *reinterpret_cast<int*>(&z_f);
    return (x * p1 xor y * p2 xor z * p3) % hash_table_size_;

  }
}