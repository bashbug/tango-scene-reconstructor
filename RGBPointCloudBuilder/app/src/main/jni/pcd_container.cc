#include "rgb-depth-sync/pcd_container.h"

namespace rgb_depth_sync {

  PCDContainer::PCDContainer(std::shared_ptr<std::mutex> pcd_mtx, std::shared_ptr<std::condition_variable> consume_pcd) {
    pcd_mtx_ = pcd_mtx;
    consume_pcd_ = consume_pcd;
    resolution_ = 0.0001; // 0.01 cm => 0.0001m
    merged_pcd_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    conversion_ = Conversion::GetInstance();
  }

  PCDContainer::~PCDContainer(){
  }

  void PCDContainer::AddPCD(PCD *pcd) {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);
    pcd_container_.push_back(pcd);
    pcl::PointCloud<pcl::PointXYZRGB> trans;
    //opengl_T_openglCamera_ = pcd->GetOpenGL_T_OpenGLCameraPose();
    //ss_T_device_ = pcd->GetPose() * conversion_->color_T_device_;
    //pcl::transformPointCloud(*(pcd->GetPCD()), trans, pcd->GetTransformationMatrix());
    //*merged_pcd_ += trans;
    /*pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (merged_pcd_);
    sor.setLeafSize (0.001f, 0.001f, 0.001f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter (*filtered);
    merged_pcd_ = filtered;*/
    consume_pcd_->notify_one();
  }

  void PCDContainer::ResetPCD() {
    pcd_container_.clear();
  }

  PCD* PCDContainer::GetLatestPCD() {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);
    if (pcd_container_.empty()) {
      return nullptr;
    } else {
      return pcd_container_.back();
    }
  }

  void PCDContainer::GetXYZRGBValues(std::vector<float>* xyz, std::vector<uint8_t>* rgb, glm::mat4* ss_T_device) {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);
    xyz_.clear();
    rgb_.clear();

    pcl::PointCloud<pcl::PointXYZRGB> trans;
    Eigen::Matrix4f ss_T_device_eigen = util::ConvertGLMToEigenPose(ss_T_device_).matrix();
    Eigen::Matrix4f device_T_ss = ss_T_device_eigen.inverse();
    pcl::transformPointCloud(*merged_pcd_, trans, device_T_ss);

    for (int i = 0; i < trans.points.size(); i++) {
      xyz_.push_back(trans.points[i].x);
      xyz_.push_back(trans.points[i].y);
      xyz_.push_back(trans.points[i].z);
      rgb_.push_back(trans.points[i].r);
      rgb_.push_back(trans.points[i].g);
      rgb_.push_back(trans.points[i].b);
    }

    *xyz = xyz_;
    *rgb = rgb_;
    *ss_T_device = ss_T_device_;
  }

  int PCDContainer::GetPCDContainerLastIndex() {
    return pcd_container_.size()-1;
  }

  std::vector<PCD*>* PCDContainer::GetPCDContainer() {
    return &pcd_container_;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCDContainer::GetMergedPCD() {
    std::unique_lock<std::mutex> lock(*pcd_mtx_);
    return merged_pcd_;
  }
} // namespace rgb_depth_syn