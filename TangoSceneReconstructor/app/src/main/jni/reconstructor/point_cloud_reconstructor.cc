#include "tango-scene-reconstructor/reconstructor/point_cloud_reconstructor.h"

namespace tango_scene_reconstructor {

  PointCloudReconstructor::PointCloudReconstructor() {
    first_ = true;
    is_running_ = false;
    pcd_mesh_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  }

  PointCloudReconstructor::~PointCloudReconstructor() {

  }

  bool PointCloudReconstructor::IsRunning() {
    return is_running_;
  }

  bool PointCloudReconstructor::Reset() {
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

  std::vector<float> PointCloudReconstructor::GetXYZValues(glm::mat4 curr_pose) {
    std::vector<float> xyz_values_transformed;
    rgb_values_.clear();

    {
      std::lock_guard<std::mutex> lock(mesh_mtx_);
      is_running_ = true;
      centroid_ = glm::vec4();
      for (int i = 0; i < pcd_mesh_->points.size(); i++) {
        glm::vec3 device_point = glm::vec3(curr_pose * glm::vec4 (pcd_mesh_->points[i].x, pcd_mesh_->points[i].y, pcd_mesh_->points[i].z, 1.0f));
        xyz_values_transformed.push_back(device_point.x);
        xyz_values_transformed.push_back(device_point.y);
        xyz_values_transformed.push_back(device_point.z);
        centroid_[0] += device_point.x;
        centroid_[1] += device_point.y;
        centroid_[2] += device_point.z;
        rgb_values_.push_back(pcd_mesh_->points[i].r);
        rgb_values_.push_back(pcd_mesh_->points[i].g);
        rgb_values_.push_back(pcd_mesh_->points[i].b);
      }

      centroid_[0] /= pcd_mesh_->points.size();
      centroid_[1] /= pcd_mesh_->points.size();
      centroid_[2] /= pcd_mesh_->points.size();
      centroid_[3] = 1.0f;

      is_running_ = false;
    }

    return xyz_values_transformed;
  }

  glm::mat4 PointCloudReconstructor::GetCentroidMatrix() {
    glm::mat4 mat = glm::mat4();
    //LOGE("CENTROID %f %f %f", centroid_[0], centroid_[1], centroid_[2]);
    mat[3][0] = centroid_[0];
    mat[3][1] = centroid_[1];
    mat[3][2] = centroid_[2];
    return mat;
  }

  std::vector<uint8_t> PointCloudReconstructor::GetRGBValues() {
    return rgb_values_;
  }

  void PointCloudReconstructor::AddPointCloudOptWithSM(PointCloud* pcd) {
    {
      std::lock_guard <std::mutex> lock(mesh_mtx_);
      is_running_ = true;

      // transfrom pcd with sm pose
      Eigen::Matrix4f transform = pcd->GetFTFSMPose().matrix();
      // add new point cloud to mesh
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*(pcd->GetPointCloud()), *transformed_cloud, transform);

      *pcd_mesh_ += *transformed_cloud;

      is_running_ = false;
    }
  }

  void PointCloudReconstructor::AddPointCloudOptWithMSM(PointCloud* pcd) {
    {
      std::lock_guard <std::mutex> lock(mesh_mtx_);
      is_running_ = true;

      // transfrom pcd with sm pose
      Eigen::Matrix4f transform = pcd->GetMFSMPose().matrix();
      // add new point cloud to mesh
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*(pcd->GetPointCloud()), *transformed_cloud, transform);

      *pcd_mesh_ += *transformed_cloud;

      is_running_ = false;
    }
  }

  void PointCloudReconstructor::DownsamplePointCloud() {
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

  void PointCloudReconstructor::FilterPointCloud(float radius) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(pcd_mesh_);
    sor.setMeanK(radius);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    pcd_mesh_ = cloud_filtered;
  }

  void PointCloudReconstructor::AddPointCloud(PointCloud* pcd) {
    {
      std::lock_guard <std::mutex> lock(mesh_mtx_);
      is_running_ = true;

      // add new point cloud to mesh
      *pcd_mesh_ += *(pcd->GetPointCloudTransformed());

      pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;

      // Create the filtering object
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud (pcd_mesh_);
      vg.setLeafSize (0.008f, 0.008f, 0.008f);
      vg.filter (cloud_downsampled);

      *pcd_mesh_ = cloud_downsampled;

      is_running_ = false;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudReconstructor::GetPCDFile() {
    return pcd_mesh_;
  }

  void PointCloudReconstructor::GenerateAndSaveMesh(std::string folderAndFileName) {
    // Normal Estimation
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> n;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (pcd_mesh_);
    n.setInputCloud (pcd_mesh_);
    n.setSearchMethod (tree);
    n.setKSearch (10);
    n.compute (*normals);
    pcl::copyPointCloud(*pcd_mesh_, *normals);
    // Greedy triangulation
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (normals);
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(0.05);
    gp3.setMu(2.0);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI);
    gp3.setMinimumAngle(M_PI / 10);
    gp3.setMaximumAngle(2 * M_PI / 3.0);
    gp3.setConsistentVertexOrdering(true);

    // Get result
    gp3.setInputCloud (normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    pcl::io::saveVTKFile (folderAndFileName.c_str(), triangles);
  }
}