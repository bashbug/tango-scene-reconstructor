#include "rgb-depth-sync/pcd.h"

namespace rgb_depth_sync {

  PCD::PCD() {
    //boost::thread* thr = new boost::thread(boost::bind(&rgb_depth_sync::PCD::foo, this));
    /*flann::Matrix<float> flann_data_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_curr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    //pcl::PointCloud2 &output;

    boost::system::error_code* error;
    boost::filesystem::path path = "/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000.pcd";
    boost::filesystem::detail::status(path, error);

    boost::shared_ptr<int> x = boost::make_shared<int>(666);

    LOGE("TEST %i", *x);

    if (pcl::io::loadPCDFile("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000.pcd", *cloud_curr) == -1) {
      LOGE("Could not open file");
    } else {
      LOGE("YEAH could open file");
    }

    //pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;setInputCloud
    pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float> > kdtree;

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
    pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> t = new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>();

    tree.reset(new pcl::search::KdTree<pcl::PointXYZRGB> (false));
    //tree->setInputCloud(cloud_curr);
    //t.clear();

    //pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
    //ne.setInputCloud (cloud_curr);
    //ne.setSearchMethod (tree);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_curr);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000000_in_.pcd", *cloud_filtered, false);

    sor.setNegative (true);
    sor.filter (*cloud_filtered);
    writer.write<pcl::PointXYZRGB> ("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000000_out_.pcd", *cloud_filtered, false);*/


    //pcl::io::savePCDFileASCII("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000000_in_.pcd", *cloud_curr);

  }

  PCD::~PCD() {
    LOGE("PointCloudData is destroyed...");
  }

  void PCD::foo() {
    LOGE("boooost");
  }

  void PCD::SetPCDWithRGBData(const std::vector<float> pcd) {
    pcd_with_rgb_data_.clear();
    xyz_values_.clear();
    rgb_values_.clear();

    for (int i = 0; i < pcd.size(); i=i+4) {
      xyz_values_.push_back(pcd[i]);
      xyz_values_.push_back(pcd[i+1]);
      xyz_values_.push_back(pcd[i+2]);
      rgb_values_.push_back(pcd[i+3]);
      pcd_with_rgb_data_.push_back(pcd[i]);
      pcd_with_rgb_data_.push_back(pcd[i+1]);
      pcd_with_rgb_data_.push_back(pcd[i+2]);
      pcd_with_rgb_data_.push_back(pcd[i+3]);
    }
  }

  void PCD::SetPCDData(const std::vector<float> pcd) {
    xyz_values_ = pcd;
  }

  void PCD::SetTranslation(const std::vector<float> translation) {
    translation_.x = translation[0];
    translation_.y = translation[1];
    translation_.z = translation[2];
  }

  void PCD::SetRotation(const std::vector<float> rotation) {
    rotation_.w = rotation[0]; //w
    rotation_.x = rotation[1]; //x
    rotation_.y = rotation[2]; //y
    rotation_.z = rotation[3]; //z
  }

  void PCD::SetPose(const glm::mat4 pose) {
    ss_T_color_ = pose;
  }

  void PCD::MapXYZWithRGB(const std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>& xyz, const std::vector<uint8_t> &rgb,
                          const double &xyz_timestamp, const double &rgb_timestamp) {

    TangoCameraIntrinsics depth_camera_intrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_DEPTH, &depth_camera_intrinsics);
    TangoCameraIntrinsics color_camera_intrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &color_camera_intrinsics);

    Conversion *conversion = Conversion::GetInstance();
    color_T_depth_ = conversion->XYZ_T_RGB(rgb_timestamp, xyz_timestamp);
    ss_T_color_ = conversion->SS_T_RGB(rgb_timestamp);
    opengl_T_color_ = conversion->OpenGL_T_RGB(ss_T_color_);
    opengl_T_openglCamera_ = conversion->OpenGL_T_OpenGLCamera(ss_T_color_);

    translation_ = util::GetTranslationFromMatrix(ss_T_color_);
    rotation_ = util::GetRotationFromMatrix(ss_T_color_);

    size_t xyz_size = xyz.size();
    size_t rgb_size = rgb.size();

    for (size_t i = 0; i < xyz_size; i++) {

      int pixel_x, pixel_y;

      // transform depth point to color frame
      glm::vec3 color_point = glm::vec3(color_T_depth_ * glm::vec4(xyz[i].x, xyz[i].y, xyz[i].z, 1.0f));

      pixel_x = static_cast<int>(color_point.x / color_point.z * color_camera_intrinsics.fx +
                                 color_camera_intrinsics.cx);

      pixel_y = static_cast<int>(color_point.y / color_point.z * color_camera_intrinsics.fy +
                                 color_camera_intrinsics.cy);

      size_t index = pixel_x + pixel_y * color_camera_intrinsics.width;

      // save rgb point cloud
      if (index * 3 + 2 >= rgb.size()) {
        //break;
        //LOGE("COLOR BAAAAAAAD INDEX: %d, size: %d", index, rgb.size());
      } else {

        pcd_with_rgb_data_.push_back(color_point.x);
        pcd_with_rgb_data_.push_back(color_point.y);
        pcd_with_rgb_data_.push_back(color_point.z);

        xyz_values_.push_back(color_point.x);
        xyz_values_.push_back(color_point.y);
        xyz_values_.push_back(color_point.z);

        rgb_values_.push_back(rgb[index * 3]);
        rgb_values_.push_back(rgb[index * 3 + 1]);
        rgb_values_.push_back(rgb[index * 3 + 2]);

        // Due to historical reasons (PCL was first developed as a ROS package), the
        // RGB information is packed into an integer and casted to a float.
        uint32_t rgb_tmp =
            ((uint32_t)(rgb[index * 3])) << 16 | ((uint32_t)(rgb[index * 3 + 1])) << 8 |
            (uint32_t)(rgb[index * 3 + 2]);
        pcd_with_rgb_data_.push_back(*reinterpret_cast<float *>(&rgb_tmp));
      }
    }
  }

  void PCD::SetKeyPointsAndDescriptors(const std::vector<cv::KeyPoint>& frame_key_points, cv::Mat frame_descriptors) {
    frame_key_points_ = frame_key_points;
    frame_descriptors_ = frame_descriptors;
  }

  void PCD::SetFrame(const cv::Mat& frame) {
    frame_ = frame;
  }

  void PCD::SetTranslation(const glm::vec3 translation) {
    translation_ = translation;
  }

  void PCD::SetRotation(const glm::quat rotation) {
    rotation_ = rotation;
  }

  glm::vec3 PCD::GetTranslation() {
    return translation_;
  }

  glm::quat PCD::GetRotation() {
    return rotation_;
  }

  std::vector<float> PCD::GetRGBPCDFileValues() {
    return rgb_values_pcd_file_;
  }

  std::vector<uint8_t> PCD::GetRGBValues() {
    return rgb_values_;
  }

  std::vector<float> PCD::GetXYZValues() {
    return xyz_values_;
  }

  std::vector<float> PCD::GetPCDData() {
    //LOGE("PointCloudData : GetPCDData");
    return pcd_with_rgb_data_;
  }
} // namespace rgb_depth_sync