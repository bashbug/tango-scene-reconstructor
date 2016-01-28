/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <future>

#include <cstring>

#include <thread>
#include <mutex>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#include <tango-gl/conversions.h>
#include <rgb-depth-sync/rgb_depth_sync_application.h>
#include "rgb-depth-sync/write_color_image.h"
#include "rgb-depth-sync/point_cloud_data.h"
#include "rgb-depth-sync/pcd_file_reader.h"
#include "rgb-depth-sync/pcd_file_writer.h"
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace g2o;


namespace rgb_depth_sync {

  /*float SynchronizationApplication::GetEuclideanDistance(const glm::vec3 curr_depth_point, const glm::vec3 trans_depth_point) {
    float distance = sqrtf(static_cast<float>((curr_depth_point[0] - trans_depth_point[0])*(curr_depth_point[0] - trans_depth_point[0]) +
                          (curr_depth_point[1] - trans_depth_point[1])*(curr_depth_point[1] - trans_depth_point[1])));
    return distance;
  }*/

  float SynchronizationApplication::GetEuclideanDistance(const glm::vec3 curr_depth_point, const glm::vec3 trans_depth_point) {
    float distance = sqrtf(static_cast<float>((curr_depth_point[0] - trans_depth_point[0])*(curr_depth_point[0] - trans_depth_point[0]) +
                                              (curr_depth_point[1] - trans_depth_point[1])*(curr_depth_point[1] - trans_depth_point[1]) +
                                              (curr_depth_point[2] - trans_depth_point[2])*(curr_depth_point[2] - trans_depth_point[2])));
    return distance;
  }

  glm::vec3 SynchronizationApplication::ConvertDepthPointTo3DPoint(const glm::vec3 depth_point) {
    glm::vec3 pcd_point;
    pcd_point[0] = static_cast<float>(depth_point.z * (depth_point.x - color_camera_intrinsics.cx) / color_camera_intrinsics.fx);
    pcd_point[1] = static_cast<float>(depth_point.z * (depth_point.y - color_camera_intrinsics.cy) / color_camera_intrinsics.fy);
    pcd_point[2] = depth_point.z;
    return pcd_point;
  }


  glm::vec3 SynchronizationApplication::Convert3DPointToDepthPoint( const glm::vec3 pcd_point) {
    glm::vec3 depth_point;
    depth_point[0] = static_cast<int>(pcd_point.x / pcd_point.z * color_camera_intrinsics.fx + color_camera_intrinsics.cx);
    depth_point[1] = static_cast<int>(pcd_point.y / pcd_point.z * color_camera_intrinsics.fy + color_camera_intrinsics.cy);
    depth_point[2] = pcd_point.z;
    return depth_point;
  }

  // Method of Horn
  inline std::vector<int> GetCirclePixels() {
    cv::Mat_<cv::Vec3b> img(720, 1280, cv::Vec3b(0,0,0));

    std::vector<int> circle_pixels;
    // 1280 x 720
    int x0 = 359;
    int y0 = 639;

    int x = 359;
    int y = 0;
    int decisionOver2 = 1 - x;   // Decision criterion divided by 2 evaluated at x=r, y=0

    while( y <= x ) {
      circle_pixels.push_back(x + x0);
      circle_pixels.push_back(y + y0); // Octant 1
      circle_pixels.push_back(y + x0);
      circle_pixels.push_back(x + y0); // Octant 2
      circle_pixels.push_back(-x + x0);
      circle_pixels.push_back(y + y0); // Octant 4
      circle_pixels.push_back(-y + x0);
      circle_pixels.push_back(x + y0); // Octant 3
      circle_pixels.push_back(-x + x0);
      circle_pixels.push_back(-y + y0); // Octant 5
      circle_pixels.push_back(-y + x0);
      circle_pixels.push_back(-x + y0); // Octant 6
      circle_pixels.push_back(x + x0);
      circle_pixels.push_back(-y + y0); // Octant 7
      circle_pixels.push_back(y + x0);
      circle_pixels.push_back(-x + y0); // Octant 8

      LOGE("Set pixel start...");

      img(x + x0,  y + y0) = cv::Vec3b(0,0,255);
      img(y + x0,  x + y0) = cv::Vec3b(0,0,255);
      img(-x + x0,  y + y0) = cv::Vec3b(0,0,255);
      img(-y + x0,  x + y0) = cv::Vec3b(0,0,255);
      img(-x + x0, -y + y0) = cv::Vec3b(0,0,255);
      img(-y + x0, -x + y0) = cv::Vec3b(0,0,255);
      img(x + x0, -y + y0) = cv::Vec3b(0,0,255);
      img(y + x0, -x + y0) = cv::Vec3b(0,0,255);

      LOGE("Set pixel stop...");

      y += 4;
      if (decisionOver2<=0)
      {
        decisionOver2 += 2 * y + 1;   // Change in decision criterion for y -> y+1
      }
      else
      {
        x -= 4;
        decisionOver2 += 2 * (y - x) + 1;   // Change for y -> y+1, x -> x-1
      }
    }

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    cv::imwrite("/storage/emulated/0/Documents/RGBPointCloudBuilder/Image/circle.jpg", img, compression_params);

    return circle_pixels;
  }

  inline void clamp(uint8_t* num) {
    if (*num < 0) {
      *num = 0;
    }
    if (*num > 255) {
      *num = 255;
    }
  }

   std::vector<float> SynchronizationApplication::GetDummyPCD() {
    std::vector<float> dummy_pcd;
    //std::vector<int> circle_pixels = GetCirclePixels();

    /*for (int i = 0; i < circle_pixels.size(); i++) {
      float x = 0.2f * (static_cast<float>(circle_pixels[i][0])-color_camera_intrinsics.cx);
      float y = 0.2f * (static_cast<float>(circle_pixels[i][1])-color_camera_intrinsics.cy);
      dummy_pcd.push_back(x);
      dummy_pcd.push_back(y);
      dummy_pcd.push_back(0.1f);
    }*/

    return dummy_pcd;
  }

  void OnFrameAvailableRouter(void* context, TangoCameraId,
                              const TangoImageBuffer* buffer) {
    SynchronizationApplication* app = static_cast<SynchronizationApplication*>(context);
    app->OnFrameAvailable(buffer);
  }

  void SynchronizationApplication::OnFrameAvailable(const TangoImageBuffer* buffer) {
    color_image_->SetImageBuffer(buffer);
    new_rgb_data_ = true;
  }

  void OnPoseAvailableRouter(void* context, const TangoPoseData* pose) {
      SynchronizationApplication* app =
          static_cast<SynchronizationApplication*>(context);
      app->OnPoseAvailable(pose);
  }

  void SynchronizationApplication::OnPoseAvailable(const TangoPoseData* pose) {
  }

  void OnXYZijAvailableRouter(void* context, const TangoXYZij* xyz_ij) {
    SynchronizationApplication* app =
        static_cast<SynchronizationApplication*>(context);
    app->OnXYZijAvailable(xyz_ij);
  }

  void SynchronizationApplication::OnXYZijAvailable(const TangoXYZij* xyz_ij) {

    // We'll just update the point cloud associated with our depth image.
    size_t point_cloud_size = xyz_ij->xyz_count * 3;
    callback_point_cloud_buffer_.resize(point_cloud_size);
    std::copy(xyz_ij->xyz[0], xyz_ij->xyz[0] + point_cloud_size,
              callback_point_cloud_buffer_.begin());
    {
      std::lock_guard<std::mutex> lock(point_cloud_mutex_);
      depth_timestamp_ = xyz_ij->timestamp;
      //LOGE("OnXYZijAvailable locked: %f", depth_timestamp_);
      callback_point_cloud_buffer_.swap(shared_point_cloud_buffer_);
      new_point_cloud_data_ = true;
    }
  }

  SynchronizationApplication::SynchronizationApplication() {
    swap_point_cloud_buffer_signal_ = false;
    render_depth_map_ = false;
    render_range_image_ = false;
    render_image_ = true;
    swap_yuv_buffer_signal_ = false;
    swap_rgb_buffer_signal_ = false;
    store_image_ = false;
    store_point_clouds_ = false;
    send_point_clouds_ = false;
    update_point_cloud_container = false;
    send_point_cloud_container = false;
    frame_available_ = false;
    id_ = 0;

    new_point_cloud_data_ = false;
    new_rgb_data_ = false;
    optimize_pose_graph_ = false;

    // We'll store the fixed transform between the opengl frame convention.
    // (Y-up, X-right) and tango frame convention. (Z-up, X-right).
    OW_T_SS_ = tango_gl::conversions::opengl_world_T_tango_world();
      LOGE("Create SynchronizationApplication");
  }

  SynchronizationApplication::~SynchronizationApplication() {
    LOGE("Destroy SynchronizationApplication");
  }

  int SynchronizationApplication::TangoInitialize(JNIEnv* env,
                                                  jobject caller_activity) {
    // The first thing we need to do for any Tango enabled application is to
    // initialize the service. We'll do that here, passing on the JNI environment
    // and jobject corresponding to the Android activity that is calling us.
    return TangoService_initialize(env, caller_activity);
  }

  int SynchronizationApplication::TangoSetPCDSave(bool isChecked) {
    store_point_clouds_ = isChecked;
    return 1;
  }

  int SynchronizationApplication::TangoSetPCDSend(bool isChecked) {
    send_point_clouds_ = isChecked;
    return 1;
  }

  int SynchronizationApplication::TangoStoreImage(bool store) {
    store_image_ = store;
    return 1;
  }

  int SynchronizationApplication::TangoSetupConfig() {
    // Default configuration enables basic motion tracking capabilities.
    tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
    if (tango_config_ == nullptr) {
      return TANGO_ERROR;
    }

    // Enable depth capturing
    TangoErrorType ret = TangoConfig_setBool(tango_config_, "config_enable_depth", true);
    if (ret != TANGO_SUCCESS) {
      LOGE("Failed to enable depth.");
      return ret;
    }

    // Note that it's super important for AR applications that we enable low
    // latency imu integration so that we have pose information available as
    // quickly as possible.
    ret = TangoConfig_setBool(tango_config_,
                              "config_enable_low_latency_imu_integration", true);
    if (ret != TANGO_SUCCESS) {
      LOGE("Failed to enable low latency imu integration.");
      return ret;
    }
    return ret;
  }

  int SynchronizationApplication::TangoConnectTexture() {
    // Connect an OpenGL texture directly to RGB camera.
    /*return TangoService_connectTextureId(TANGO_CAMERA_COLOR, color_image_->GetTextureId(),
                                         this, nullptr);*/
  }

  int SynchronizationApplication::TangoConnectCallbacks() {

    // Attach OnXYZijAvailable callback.
    int ret = TangoService_connectOnXYZijAvailable(OnXYZijAvailableRouter);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to connect to point cloud callback with error "
               "code: %d", ret);
      return ret;
    }

    // Attach OnFrameAvailable callback.
    ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this, OnFrameAvailableRouter);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to connect to on frame available callback with error"
               "code: %d", ret);
      return ret;
    }

    // Setting up the frame pair for the OnPoseAvailable callback.
    TangoCoordinateFramePair pairs;
    pairs.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    pairs.target = TANGO_COORDINATE_FRAME_DEVICE;

    // Attach OnPoseAvailable callback.
    ret = TangoService_connectOnPoseAvailable(1, &pairs, OnPoseAvailableRouter);
    if (ret != TANGO_SUCCESS) {
      LOGE("MotiongTrackingApp: Failed to connect to pose callback with error"
               "code: %d", ret);
      return ret;
    }
  }

  int SynchronizationApplication::TangoConnect() {
    TangoErrorType ret = TangoService_connect(this, tango_config_);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to connect to the Tango service.");
    }
    return ret;
  }

  int SynchronizationApplication::TangoSetIntrinsicsAndExtrinsics() {

    /*TangoErrorType ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_COLOR, &color_camera_intrinsics);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get the intrinsics for the color"
          "camera.");
      return ret;
    }*/

    //dummy_pcd_ = GetDummyPCD();
    //circle_pixels_ = GetCirclePixels();
    //LOGE("How many points: %i", circle_pixels_.size());

    range_image_->SetCameraIntrinsics(color_camera_intrinsics);
    //scan_matcher_->SetCameraIntrinsics(color_camera_intrinsics);

    float image_width = static_cast<float>(color_camera_intrinsics.width);
    float image_height = static_cast<float>(color_camera_intrinsics.height);
    float image_plane_ratio = image_height / image_width;
    float screen_ratio = screen_height_ / screen_width_;

    if (image_plane_ratio < screen_ratio) {
      glViewport(0, 0, screen_width_, screen_width_ * image_plane_ratio);
    } else {
      glViewport((screen_width_ - screen_height_ / image_plane_ratio) / 2, 0,
                 screen_height_ / image_plane_ratio, screen_height_);
    }

    TangoPoseData pose_imu_T_device;
    TangoPoseData pose_imu_T_color;
    TangoPoseData pose_imu_T_depth;
    TangoCoordinateFramePair frame_pair;

    // Extrinsic transformation between the device and the imu coordinate frame.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoErrorType ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_device);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get transform between the IMU "
          "and"
          "device frames. Something is wrong with device extrinsics.");
      return ret;
    }

    // Extrinsic transformation between the color camera and the imu coordinate frame.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_color);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get transform between the IMU "
          "and"
          "color camera frames. Something is wrong with device extrinsics.");
      return ret;
    }

    // Extrinsic transformation between the depth camera and the imu coordinate frame.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_depth);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get transform between the IMU "
          "and depth camera frames. Something is wrong with device extrinsics.");
      return ret;
    }

    glm::mat4 imu_T_device = util::GetMatrixFromPose(&pose_imu_T_device);
    glm::mat4 imu_T_color = util::GetMatrixFromPose(&pose_imu_T_color);
    glm::mat4 imu_T_depth = util::GetMatrixFromPose(&pose_imu_T_depth);

    glm::mat4 device_T_imu = glm::inverse(imu_T_device);

    device_T_color_ = glm::inverse(imu_T_device) * imu_T_color;
    device_T_depth_ = glm::inverse(imu_T_device) * imu_T_depth;
    color_T_device_ = glm::inverse(device_T_color_);
    glm::mat4 color_T_depth = color_T_device_ * device_T_depth_;

    return ret;
  }

  void SynchronizationApplication::TangoDisconnect() {
    TangoConfig_free(tango_config_);
    tango_config_ = nullptr;
    TangoService_disconnect();
  }

  void SynchronizationApplication::SetViewPort(int width, int height) {
    screen_width_ = static_cast<float>(width);
    screen_height_ = static_cast<float>(height);
    scene_->SetViewPort(width, height);
  }

  std::vector<PointCloudData*>& SynchronizationApplication::allPCD() {
    static std::vector<PointCloudData*> all_pcd;
    return all_pcd;
  }

  void SynchronizationApplication::InitializeGLContent() {
    color_image_ = new rgb_depth_sync::ColorImage();
    range_image_ = new rgb_depth_sync::RangeImage();
    point_cloud_container_ = new rgb_depth_sync::PointCloudContainer();
    pcd_file_reader_ = new rgb_depth_sync::PCDFileReader();
    pcd_file_writer_ = new rgb_depth_sync::PCDFileWriter();
    scene_ = new rgb_depth_sync::Scene();
    slam_ = new rgb_depth_sync::Slam3D();
    scan_matcher_ = new rgb_depth_sync::ScanMatcher();

    color_timestamp_ = 0.0;
    depth_timestamp_ = 0.0;

    static int count = 0;

    TangoErrorType ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_COLOR, &color_camera_intrinsics);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get the intrinsics for the color"
              "camera.");
    }

    for (int i = 0; i <= 196;  i++) {
      char filename[1024];
      sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Loop_banana/%05d.pcd", i);
      pcd_file_reader_->ReadFile(filename);
      PointCloudData* pcd = new rgb_depth_sync::PointCloudData(i);
      pcd->SetPCDWithRGBData(pcd_file_reader_->GetPointsWithRGB());
      pcd->SetTranslation(pcd_file_reader_->GetTranslation());
      pcd->SetRotation(pcd_file_reader_->GetRotation());

      point_cloud_container_->SetPointCloudData(pcd);

      glm::vec3 translation = pcd->GetTranslation();
      glm::quat rotation = pcd->GetRotation();

      glm::mat4 pose_matrix = glm::mat4_cast(rotation);
      pose_matrix[3][0] = pcd->GetTranslation()[0];
      pose_matrix[3][1] = pcd->GetTranslation()[1];
      pose_matrix[3][2] = pcd->GetTranslation()[2];
      pose_matrix[3][3] = 1;

      pcd->SetPose(pose_matrix);

      Eigen::Isometry3f odometryPose = util::ConvertGLMToEigenPose(pose_matrix);
      Eigen::Isometry3d odometryPose_d = util::CastIsometry3fTo3d(odometryPose);

      // add node to the pose graph
      id_ = slam_->AddNode(odometryPose_d);
      // add edge to the pose graph
      if(id_ > 0) {
        slam_->AddEdge(id_-1, id_);
      }
    }

    typedef std::pair<int, int> key;
    typedef std::future<Eigen::Isometry3f> value;
    std::map<key, value> poses;
    std::map<key, value>::iterator it;

    // 0-7 1-8 ... 6-13
    int size = point_cloud_container_->GetPointCloudContainer().size();

    glm::vec3 curr_depth_point_lu = glm::vec3(119.0f, 119.0f, 0.5f);
    glm::vec4 curr_pcd_point_lu = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_lu),
                                            0.1f);
    glm::vec3 curr_depth_point_ld = glm::vec3(119.0f, 599.0f, 0.5f);
    glm::vec4 curr_pcd_point_ld = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_ld),
                                            0.1f);
    glm::vec3 curr_depth_point_cc = glm::vec3(639.0f, 359.0f, 0.5f);
    glm::vec4 curr_pcd_point_cc = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_cc),
                                            0.1f);
    glm::vec3 curr_depth_point_ru = glm::vec3(1159.0f, 119.0f, 0.5f);
    glm::vec4 curr_pcd_point_ru = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_ru),
                                            0.1f);
    glm::vec3 curr_depth_point_rd = glm::vec3(1159.0f, 599.0f, 0.5f);
    glm::vec4 curr_pcd_point_rd = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_rd),
                                            0.1f);
    for (int k = 0 ; k < size; k++) {
      //LOGE("frame search: %i", k);
      //LOGE("Check for loop closure frames start...");
      glm::mat4 curr_pose = glm::inverse(
          point_cloud_container_->GetPointCloudContainer()[k]->GetPose());
      // set central point with a range of 1 meter
      // set central point with a range of 1 meter

      //LOGE("curr_pcd: x %f, y %f, z %f", curr_pcd_point[0], curr_pcd_point[1],curr_pcd_point[2]);
      //LOGE("Check loop closure frames for frame %i", point_cloud_container_->GetPointCloudContainer().size());
      for (int i = 0; i < size; i++) {
        //LOGE("frame: %i", i);
        // transform the pcd point from the current pose to all other frames
        //LOGE("curr_pcd: x %f, y %f, z %f", curr_pcd_point[0], curr_pcd_point[1],curr_pcd_point[2]);

        if (i == k) {
          continue;
        }
        glm::mat4 relativ_transformation =
            curr_pose * point_cloud_container_->GetPointCloudContainer()[i]->GetPose();

        //glm::vec3 translation = util::GetTranslationFromMatrix(relativ_transformation);
        //glm::quat rotation = util::GetRotationFromMatrix(relativ_transformation);

        //LOGE("RELATIVE TRANS: x %f, y %f, z %f", translation.x, translation.y, translation.z);
        //LOGE("RELATIVE ROT: x %f, y %f, z %f, w %f", rotation.x, rotation.y, rotation.z, rotation.w);
        // transform the pcd point from the current pose to all other frames
        glm::vec4 trans_pcd_point_lu = relativ_transformation * curr_pcd_point_lu;
        float distance = GetEuclideanDistance(
            glm::vec3(curr_pcd_point_lu[0], curr_pcd_point_lu[1], curr_pcd_point_lu[2]),
            glm::vec3(trans_pcd_point_lu[0], trans_pcd_point_lu[1], trans_pcd_point_lu[2]));
        float distance_accum = 0;
        int distance_accum_c = 0;
        if (distance <= 0.1f) {
          distance_accum += distance;
          distance_accum_c++;
          //LOGE("Frame: %i, lu distance: %f", i, distance);
        }

        glm::vec4 trans_pcd_point_ld = relativ_transformation * curr_pcd_point_ld;
        distance = GetEuclideanDistance(
            glm::vec3(curr_pcd_point_ld[0], curr_pcd_point_ld[1], curr_pcd_point_ld[2]),
            glm::vec3(trans_pcd_point_ld[0], trans_pcd_point_ld[1], trans_pcd_point_ld[2]));

        if (distance <= 0.1f) {
          distance_accum += distance;
          distance_accum_c++;
          //LOGE("Frame: %i, ld distance: %f", i, distance);
        }

        glm::vec4 trans_pcd_point_cc = relativ_transformation * curr_pcd_point_cc;
        distance = GetEuclideanDistance(
            glm::vec3(curr_pcd_point_cc[0], curr_pcd_point_cc[1], curr_pcd_point_cc[2]),
            glm::vec3(trans_pcd_point_cc[0], trans_pcd_point_cc[1], trans_pcd_point_cc[2]));

        if (distance <= 0.1f) {
          distance_accum += distance;
          distance_accum_c++;
          //LOGE("Frame: %i, cc distance: %f", i, distance);
        }

        glm::vec4 trans_pcd_point_ru = relativ_transformation * curr_pcd_point_ru;
        distance = GetEuclideanDistance(
            glm::vec3(curr_pcd_point_ru[0], curr_pcd_point_ru[1], curr_pcd_point_ru[2]),
            glm::vec3(trans_pcd_point_ru[0], trans_pcd_point_ru[1], trans_pcd_point_ru[2]));

        if (distance <= 0.1f) {
          distance_accum += distance;
          distance_accum_c++;
          //LOGE("Frame: %i, ru distance: %f", i, distance);
        }

        glm::vec4 trans_pcd_point_rd = relativ_transformation * curr_pcd_point_rd;
        distance = GetEuclideanDistance(
            glm::vec3(curr_pcd_point_rd[0], curr_pcd_point_rd[1], curr_pcd_point_rd[2]),
            glm::vec3(trans_pcd_point_rd[0], trans_pcd_point_rd[1], trans_pcd_point_rd[2]));

        if (distance <= 0.1f) {
          distance_accum += distance;
          distance_accum_c++;
          //LOGE("Frame: %i, rd distance: %f", i, distance);
        }

        if (distance_accum_c > 0) {
          float distance_average = 0;
          distance_average = distance_accum / distance_accum_c;
          if (distance_average <= 0.5f) {
            //LOGE("Frame %i has a distance average of %f", i, distance_average);
            ScanMatcher* scan_matcher = new rgb_depth_sync::ScanMatcher();
            std::future <Eigen::Isometry3f> loop_pose_async = std::async(std::launch::async,
                                                                         &rgb_depth_sync::ScanMatcher::Match,
                                                                         *scan_matcher,
                                                                         point_cloud_container_->GetPointCloudContainer()[i]->GetXYZValues_T_Color(),
                                                                         point_cloud_container_->GetPointCloudContainer()[k]->GetXYZValues_T_Color(),
                                                                         slam_->GetPose(
                                                                             point_cloud_container_->GetPointCloudContainer()[i]->GetId()),
                                                                         slam_->GetPose(
                                                                             point_cloud_container_->GetPointCloudContainer()[k]->GetId()));

            poses.insert(
                std::pair<key, value>(std::pair<int, int>(i, k), std::move(loop_pose_async)));

            count++;
          }
        }
      }
    }
      LOGE("%i loops found", count);
      LOGE("SynchronizationApplication : Scan matcher start...");
      for (it = poses.begin(); it != poses.end(); it++) {
        slam_->AddLoopClosure(it->first.first, it->first.second, it->second.get(), 10);
      }
      LOGE("SynchronizationApplication : Scan matcher stop...");
      slam_->SaveGraph();
      LOGE("SynchronizationApplication : optimize graph start...");
      slam_->OptimizeGraph();
      LOGE("SynchronizationApplication : optimize graph stop...");
      slam_->SaveGraph();
      std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> all_poses = slam_->GetPoses();

    /*for (int i = 0; i < point_cloud_container_->GetPointCloudContainer().size(); i++) {
      pcd_file_writer_->SetPCDRGBData(
          point_cloud_container_->GetPointCloudContainer()[i]->GetPCDData(),
          point_cloud_container_->GetPointCloudContainer()[i]->GetTranslation(),
          point_cloud_container_->GetPointCloudContainer()[i]->GetRotation());
      pcd_file_writer_->SetUnordered();
      pcd_file_writer_->SaveToFile("Loop_book_before", i);
    }*/

    for (int i = 0; i < all_poses.size(); i++) {
      glm::mat4 icppose_glm = util::ConvertEigenToGLMPose(all_poses[i]);
      glm::vec3 icp_translation = util::GetTranslationFromMatrix(icppose_glm);
      glm::quat icp_rotation = util::GetRotationFromMatrix(icppose_glm);

      //icp_positions[i] = icp_translation;
      point_cloud_container_->GetPointCloudContainer()[i]->SetTranslation(icp_translation);
      point_cloud_container_->GetPointCloudContainer()[i]->SetRotation(icp_rotation);
    }

    LOGE("SynchronizationApplication : Save %i files after optimization start...", point_cloud_container_->GetPointCloudContainer().size());

    for (int i = 0; i < point_cloud_container_->GetPointCloudContainer().size(); i++) {
      rgb_depth_sync::PCDFileWriter pcd_file_writer;
      pcd_file_writer.SetPCDRGBData(
          point_cloud_container_->GetPointCloudContainer()[i]->GetPCDData(),
          point_cloud_container_->GetPointCloudContainer()[i]->GetTranslation(),
          point_cloud_container_->GetPointCloudContainer()[i]->GetRotation());
      pcd_file_writer.SetUnordered();
      std::async(std::launch::async, &rgb_depth_sync::PCDFileWriter::SaveToFile, pcd_file_writer, "Loop_banana_opt", i);
    }

    //LOGE("Check for loop closure frames start...");
    /*if (point_cloud_container_->GetLatestPCD() != nullptr) {

      int size = point_cloud_container_->GetPointCloudContainer().size();

      for (int k = 0; k < size; k++) {
        glm::mat4 curr_pose = glm::inverse(point_cloud_container_->GetPointCloudContainer()[k]->GetPose());

        // set central point with a range of 1 meter
        glm::vec3 curr_depth_point_lu = glm::vec3(119.0f, 119.0f, 0.5f);
        glm::vec4 curr_pcd_point_lu = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_lu), 0.1f);
        glm::vec3 curr_depth_point_ld = glm::vec3(119.0f, 599.0f, 0.5f);
        glm::vec4 curr_pcd_point_ld = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_ld), 0.1f);
        glm::vec3 curr_depth_point_cc = glm::vec3(639.0f, 359.0f, 0.5f);
        glm::vec4 curr_pcd_point_cc = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_cc), 0.1f);
        glm::vec3 curr_depth_point_ru = glm::vec3(1159.0f, 119.0f, 0.5f);
        glm::vec4 curr_pcd_point_ru = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_ru), 0.1f);
        glm::vec3 curr_depth_point_rd = glm::vec3(1159.0f, 599.0f, 0.5f);
        glm::vec4 curr_pcd_point_rd = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_rd), 0.1f);
        //LOGE("curr_pcd: x %f, y %f, z %f", curr_pcd_point[0], curr_pcd_point[1],curr_pcd_point[2]);
        LOGE("Frame %i check loop...", k);
        for (int i = 0; i < size; i++) {
          if(k == i) {
            continue;
          }
          glm::mat4 relativ_transformation =
              curr_pose * point_cloud_container_->GetPointCloudContainer()[i]->GetPose();

          glm::vec3 translation = util::GetTranslationFromMatrix(relativ_transformation);
          glm::quat rotation = util::GetRotationFromMatrix(relativ_transformation);

          //LOGE("RELATIVE TRANS: x %f, y %f, z %f", translation.x, translation.y, translation.z);
          //LOGE("RELATIVE ROT: x %f, y %f, z %f, w %f", rotation.x, rotation.y, rotation.z, rotation.w);
          // transform the pcd point from the current pose to all other frames
          glm::vec4 trans_pcd_point_lu = relativ_transformation * curr_pcd_point_lu;
          float distance = GetEuclideanDistance(
              glm::vec3(curr_pcd_point_lu[0],curr_pcd_point_lu[1],curr_pcd_point_lu[2]),
              glm::vec3(trans_pcd_point_lu[0],trans_pcd_point_lu[1], trans_pcd_point_lu[2]));
          LOGE("Frame: %i, lu distance: %f", i, distance);

          glm::vec4 trans_pcd_point_ld = relativ_transformation * curr_pcd_point_ld;
          distance = GetEuclideanDistance(
              glm::vec3(curr_pcd_point_ld[0],curr_pcd_point_ld[1],curr_pcd_point_ld[2]),
              glm::vec3(trans_pcd_point_ld[0],trans_pcd_point_ld[1], trans_pcd_point_ld[2]));
          LOGE("Frame: %i, ld distance: %f", i, distance);

          glm::vec4 trans_pcd_point_cc = relativ_transformation * curr_pcd_point_cc;
          distance = GetEuclideanDistance(
              glm::vec3(curr_pcd_point_cc[0],curr_pcd_point_cc[1],curr_pcd_point_cc[2]),
              glm::vec3(trans_pcd_point_cc[0],trans_pcd_point_cc[1], trans_pcd_point_cc[2]));
          LOGE("Frame: %i, cc distance: %f", i, distance);

          glm::vec4 trans_pcd_point_ru = relativ_transformation * curr_pcd_point_ru;
          distance = GetEuclideanDistance(
              glm::vec3(curr_pcd_point_ru[0],curr_pcd_point_ru[1],curr_pcd_point_ru[2]),
              glm::vec3(trans_pcd_point_ru[0],trans_pcd_point_ru[1], trans_pcd_point_ru[2]));
          LOGE("Frame: %i, ru distance: %f", i, distance);

          glm::vec4 trans_pcd_point_rd = relativ_transformation * curr_pcd_point_rd;
          distance = GetEuclideanDistance(
              glm::vec3(curr_pcd_point_rd[0],curr_pcd_point_rd[1],curr_pcd_point_rd[2]),
              glm::vec3(trans_pcd_point_rd[0],trans_pcd_point_rd[1], trans_pcd_point_rd[2]));
          LOGE("Frame: %i, rd distance: %f", i, distance);

          //glm::vec4 trans_pcd_point_ld = relativ_transformation * curr_pcd_point_ld;
          //glm::vec4 trans_pcd_point_cc = relativ_transformation * curr_pcd_point_cc;
          //glm::vec4 trans_pcd_point_ru = relativ_transformation * curr_pcd_point_ru;
          //glm::vec4 trans_pcd_point_rd = relativ_transformation * curr_pcd_point_rd;

          //glm::vec3 tmp(trans_pcd_point[0], trans_pcd_point[1], trans_pcd_point[2]);
          // get pixel coordinates of trans_pcd_point
          //LOGE("trans_pcd: x %f, y %f, z %f", tmp[0], tmp[1], tmp[2]);
          /*glm::vec3 trans_depth_point_lu = Convert3DPointToDepthPoint(glm::vec3(trans_pcd_point_lu[0], trans_pcd_point_lu[1], trans_pcd_point_lu[2]));
          glm::vec3 trans_depth_point_ld = Convert3DPointToDepthPoint(glm::vec3(trans_pcd_point_ld[0], trans_pcd_point_ld[1], trans_pcd_point_ld[2]));
          glm::vec3 trans_depth_point_cc = Convert3DPointToDepthPoint(glm::vec3(trans_pcd_point_cc[0], trans_pcd_point_cc[1], trans_pcd_point_cc[2]));
          glm::vec3 trans_depth_point_ru = Convert3DPointToDepthPoint(glm::vec3(trans_pcd_point_ru[0], trans_pcd_point_ru[1], trans_pcd_point_ru[2]));
          glm::vec3 trans_depth_point_rd = Convert3DPointToDepthPoint(glm::vec3(trans_pcd_point_rd[0], trans_pcd_point_rd[1], trans_pcd_point_rd[2]));

          //LOGE("trans_pixel: x %f, y %f", trans_depth_point[0], trans_depth_point[1]);
          // check if point is out of the image borders
          if (trans_depth_point_lu[0] > 0 &&
              trans_depth_point_lu[0] < color_camera_intrinsics.width &&
              trans_depth_point_lu[1] > 0 &&
              trans_depth_point_lu[1] < color_camera_intrinsics.height) {
            float distance = GetEuclideanDistance(curr_depth_point_lu, trans_depth_point_lu);
            /*if (trans_pcd_point_lu[2] > 2.0f || trans_pcd_point_lu[2] < 0.05f) {
              //LOGE("Frame: %i, distance: %f but z: %f not in frustum", i, distance, trans_pcd_point[2]);
            } else if (distance > 150) {
              //LOGE("Frame: %i, distance is to big: %f", i, distance);
            } else {
              LOGE("Frame: %i, distance: %f", i, distance);

              // get icp pose from scan matcher
              //LOGE("Get loop pose start...");
              /*Eigen::Isometry3f loop_pose = scan_matcher_->Match(
                  point_cloud_container_->GetPointCloudContainer()[i]->GetXYZValues(),
                  point_cloud_container_->GetPointCloudContainer()[point_cloud_container_->GetLatestPCD()->GetId()]->GetXYZValues(),
                  slam_->GetPose(
                      point_cloud_container_->GetPointCloudContainer()[i]->GetId()),
                  slam_->GetPose(
                      point_cloud_container_->GetPointCloudContainer()[point_cloud_container_->GetLatestPCD()->GetId()]->GetId()));

              Eigen::Isometry3d relative_pose = util::CastIsometry3fTo3d(loop_pose);
              //LOGE("Get loop pose stop...");

              //LOGE("Add loop closure...");
              int confidence = 160 / distance + 1;
              slam_->AddLoopClosure(
                  point_cloud_container_->GetPointCloudContainer()[i]->GetId(),
                  point_cloud_container_->GetPointCloudContainer()[point_cloud_container_->GetLatestPCD()->GetId()]->GetId(),
                  relative_pose, confidence);

            //LOGE("frame %i out of bound", i);
          }


          if (trans_depth_point_ld[0] > 0 &&
              trans_depth_point_ld[0] < color_camera_intrinsics.width &&
              trans_depth_point_ld[1] > 0 &&
              trans_depth_point_ld[1] < color_camera_intrinsics.height) {
            float distance = GetEuclideanDistance(curr_depth_point_ld, trans_depth_point_ld);
            /*if (trans_pcd_point_ld[2] > 2.0f || trans_pcd_point_ld[2] < 0.05f) {
              //LOGE("Frame: %i, distance: %f but z: %f not in frustum", i, distance, trans_pcd_point[2]);
            } else if (distance > 150) {
              //LOGE("Frame: %i, distance is to big: %f", i, distance);
            } else {
              LOGE("Frame: %i, distance: %f", i, distance);
            }


          if (trans_depth_point_cc[0] > 0 &&
              trans_depth_point_cc[0] < color_camera_intrinsics.width &&
              trans_depth_point_cc[1] > 0 &&
              trans_depth_point_cc[1] < color_camera_intrinsics.height) {
            float distance = GetEuclideanDistance(curr_depth_point_cc, trans_depth_point_cc);
            /*if (trans_pcd_point_cc[2] > 2.0f || trans_pcd_point_cc[2] < 0.05f) {
              //LOGE("Frame: %i, distance: %f but z: %f not in frustum", i, distance, trans_pcd_point[2]);
            } else if (distance > 150) {
              //LOGE("Frame: %i, distance is to big: %f", i, distance);
            } else {
              LOGE("Frame: %i, distance: %f", i, distance);
            }


          if (trans_depth_point_ru[0] > 0 &&
              trans_depth_point_ru[0] < color_camera_intrinsics.width &&
              trans_depth_point_ru[1] > 0 &&
              trans_depth_point_ru[1] < color_camera_intrinsics.height) {
            float distance = GetEuclideanDistance(curr_depth_point_ru, trans_depth_point_ru);
            /*if (trans_pcd_point_ru[2] > 2.0f || trans_pcd_point_ru[2] < 0.05f) {
              //LOGE("Frame: %i, distance: %f but z: %f not in frustum", i, distance, trans_pcd_point[2]);
            } else if (distance > 150) {
              //LOGE("Frame: %i, distance is to big: %f", i, distance);
            } else {
              LOGE("Frame: %i, distance: %f", i, distance);
            }


          if (trans_depth_point_rd[0] > 0 &&
              trans_depth_point_rd[0] < color_camera_intrinsics.width &&
              trans_depth_point_rd[1] > 0 &&
              trans_depth_point_rd[1] < color_camera_intrinsics.height) {
            float distance = GetEuclideanDistance(curr_depth_point_rd, trans_depth_point_rd);
            /*if (trans_pcd_point_rd[2] > 2.0f || trans_pcd_point_rd[2] < 0.05f) {
              //LOGE("Frame: %i, distance: %f but z: %f not in frustum", i, distance, trans_pcd_point[2]);
            } else if (distance > 150) {
              //LOGE("Frame: %i, distance is to big: %f", i, distance);
            } else {
              LOGE("Frame: %i, distance: %f", i, distance);
            }


        }
      }
    }*/

  }

  void SynchronizationApplication::Render() {
    double rgb_timestamp = 0.0;
    double color_timestamp = 0.0;
    double depth_timestamp = 0.0;

    static bool pcd_available = false;
    static bool rgb_available = false;
    static bool pcd_rendering = false;
    //static int count = 0;

    {
      std::lock_guard<std::mutex> lock(point_cloud_mutex_);
      depth_timestamp = depth_timestamp_;
      if (new_point_cloud_data_) {
        //LOGE("Depth locked : %f", depth_timestamp_);
        shared_point_cloud_buffer_.swap(render_point_cloud_buffer_);
        new_point_cloud_data_ = false;
        pcd_available = true;
        pcd_rendering = true;
      } else {
        pcd_available = false;
      }
    }

    if (new_rgb_data_ && pcd_rendering) {
      int lock = pthread_mutex_trylock(&(color_image_->color_image_mutex_));
      //color_timestamp = color_timestamp_;
      if (lock == 0) {
        rgb_buffer_.clear();
        rgb_buffer_.resize(1280 * 720 * 3);
        cv::Mat tmp;
        tmp.create(720, 1280, CV_8UC3);
        color_image_->GetImage(depth_timestamp, &tmp, &color_timestamp);
        memcpy(&rgb_buffer_[0], tmp.data, 1280 * 720 * 3);
        color_timestamp_ = color_timestamp;
        //LOGE("RGB locked : %f", color_timestamp_);
        rgb_available = true;
        pthread_mutex_unlock(&(color_image_->color_image_mutex_));
      }
    }

    if (new_rgb_data_ && pcd_rendering) {
      // Querying the depth image's frame transformation based on the depth image's
      // timestamp.
      TangoPoseData pose_start_service_T_device_t0;
      TangoCoordinateFramePair depth_frame_pair;
      depth_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
      depth_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
      if (TangoService_getPoseAtTime(depth_timestamp, depth_frame_pair,
                                     &pose_start_service_T_device_t0) !=
          TANGO_SUCCESS) {
        LOGE(
            "SynchronizationApplication: Could not find a valid pose at time %lf"
                " for the depth camera.",
            depth_timestamp);
      }

      // Querying the color image's frame transformation based on the depth image's
      // timestamp.
      TangoPoseData pose_start_service_T_device_t1;
      TangoCoordinateFramePair color_frame_pair;
      color_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
      color_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
      if (TangoService_getPoseAtTime(color_timestamp_, color_frame_pair,
                                     &pose_start_service_T_device_t1) !=
          TANGO_SUCCESS) {
        LOGE(
            "SynchronizationApplication: Could not find a valid pose at time %lf"
                " for the color camera.",
            color_timestamp_);
      }

      if (pose_start_service_T_device_t1.status_code == TANGO_POSE_VALID) {
        if (pose_start_service_T_device_t0.status_code == TANGO_POSE_VALID) {

          glm::mat4 start_service_T_device_t0 = util::GetMatrixFromPose(
              &pose_start_service_T_device_t0);
          glm::mat4 start_service_T_device_t1 = util::GetMatrixFromPose(
              &pose_start_service_T_device_t1);
          glm::mat4 device_t0_T_depth_t0 = device_T_depth_;
          glm::mat4 color_t1_T_device_t1 = glm::inverse(device_T_color_);
          glm::mat4 start_service_T_color_t1 = start_service_T_device_t1 * device_T_color_;
          glm::mat4 start_service_T_depth_t0 = start_service_T_device_t0 * device_T_depth_;

          glm::mat4 color_T_depth =
              color_t1_T_device_t1 *
              glm::inverse(start_service_T_device_t1) *
              start_service_T_device_t0 *
              device_t0_T_depth_t0;

          glm::mat4 open_gl_T_cur_pose_T_gl_camera, open_gl_T_cur_pose, icppose_glm;

          open_gl_T_cur_pose_T_gl_camera =
              tango_gl::conversions::opengl_world_T_tango_world() *
              start_service_T_color_t1 * tango_gl::conversions::depth_camera_T_opengl_camera();

          open_gl_T_cur_pose =
              tango_gl::conversions::opengl_world_T_tango_world() *
              start_service_T_color_t1;

          //LOGE("diff :  %f", fabs(depth_timestamp - color_timestamp_));
          if (store_image_ && pcd_available && rgb_available) {

            // add pose to pose graph optimizer
            Eigen::Isometry3f odometryPose = util::ConvertGLMToEigenPose(start_service_T_color_t1);
            Eigen::Isometry3d odometryPose_d = util::CastIsometry3fTo3d(odometryPose);
            // add node to the pose graph
            id_ = slam_->AddNode(odometryPose_d);
            // add edge to the pose graph
            if (id_ > 0) {
              slam_->AddEdge(id_ - 1, id_);
            }

            // use returned id to init a new pcd object
            PointCloudData *pcd = new rgb_depth_sync::PointCloudData(id_);

            icppose_glm = glm::mat4();

            pcd->SetCameraIntrinsics(color_camera_intrinsics);

            pcd->MapXYZWithRGB(start_service_T_color_t1,
                               start_service_T_depth_t0,
                               color_T_depth,
                               open_gl_T_cur_pose,
                               shared_point_cloud_buffer_,
                               rgb_buffer_);
            point_cloud_container_->SetPointCloudData(pcd);

            //LOGE("Check for loop closure frames start...");
            if (point_cloud_container_->GetLatestPCD() != nullptr) {
              glm::mat4 curr_pose = glm::inverse(point_cloud_container_->GetLatestPCD()->GetPose());
              size_t size = point_cloud_container_->GetPointCloudContainer().size();
              // set central point with a range of 1 meter
              // set central point with a range of 1 meter
              glm::vec3 curr_depth_point_lu = glm::vec3(119.0f, 119.0f, 0.5f);
              glm::vec4 curr_pcd_point_lu = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_lu), 0.1f);
              glm::vec3 curr_depth_point_ld = glm::vec3(119.0f, 599.0f, 0.5f);
              glm::vec4 curr_pcd_point_ld = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_ld), 0.1f);
              glm::vec3 curr_depth_point_cc = glm::vec3(639.0f, 359.0f, 0.5f);
              glm::vec4 curr_pcd_point_cc = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_cc), 0.1f);
              glm::vec3 curr_depth_point_ru = glm::vec3(1159.0f, 119.0f, 0.5f);
              glm::vec4 curr_pcd_point_ru = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_ru), 0.1f);
              glm::vec3 curr_depth_point_rd = glm::vec3(1159.0f, 599.0f, 0.5f);
              glm::vec4 curr_pcd_point_rd = glm::vec4(ConvertDepthPointTo3DPoint(curr_depth_point_rd), 0.1f);

              //LOGE("curr_pcd: x %f, y %f, z %f", curr_pcd_point[0], curr_pcd_point[1],curr_pcd_point[2]);
              LOGE("Check loop closure frames for frame %i", point_cloud_container_->GetPointCloudContainer().size());
              for (int i = 0; i < point_cloud_container_->GetPointCloudContainer().size() - 1; i++) {
                // transform the pcd point from the current pose to all other frames
                //LOGE("curr_pcd: x %f, y %f, z %f", curr_pcd_point[0], curr_pcd_point[1],curr_pcd_point[2]);

                glm::mat4 relativ_transformation =
                    curr_pose * point_cloud_container_->GetPointCloudContainer()[i]->GetPose();

                //glm::vec3 translation = util::GetTranslationFromMatrix(relativ_transformation);
                //glm::quat rotation = util::GetRotationFromMatrix(relativ_transformation);

                //LOGE("RELATIVE TRANS: x %f, y %f, z %f", translation.x, translation.y, translation.z);
                //LOGE("RELATIVE ROT: x %f, y %f, z %f, w %f", rotation.x, rotation.y, rotation.z, rotation.w);
                // transform the pcd point from the current pose to all other frames
                glm::vec4 trans_pcd_point_lu = relativ_transformation * curr_pcd_point_lu;
                float distance = GetEuclideanDistance(
                    glm::vec3(curr_pcd_point_lu[0],curr_pcd_point_lu[1],curr_pcd_point_lu[2]),
                    glm::vec3(trans_pcd_point_lu[0],trans_pcd_point_lu[1], trans_pcd_point_lu[2]));
                float distance_accum = 0;
                int distance_accum_c = 0;
                if (distance <= 0.1f) {
                  distance_accum += distance;
                  distance_accum_c++;
                  //LOGE("Frame: %i, lu distance: %f", i, distance);
                }

                glm::vec4 trans_pcd_point_ld = relativ_transformation * curr_pcd_point_ld;
                distance = GetEuclideanDistance(
                    glm::vec3(curr_pcd_point_ld[0],curr_pcd_point_ld[1],curr_pcd_point_ld[2]),
                    glm::vec3(trans_pcd_point_ld[0],trans_pcd_point_ld[1], trans_pcd_point_ld[2]));

                if (distance <= 0.1f) {
                  distance_accum += distance;
                  distance_accum_c++;
                  //LOGE("Frame: %i, ld distance: %f", i, distance);
                }

                glm::vec4 trans_pcd_point_cc = relativ_transformation * curr_pcd_point_cc;
                distance = GetEuclideanDistance(
                    glm::vec3(curr_pcd_point_cc[0],curr_pcd_point_cc[1],curr_pcd_point_cc[2]),
                    glm::vec3(trans_pcd_point_cc[0],trans_pcd_point_cc[1], trans_pcd_point_cc[2]));

                if (distance <= 0.1f) {
                  distance_accum += distance;
                  distance_accum_c++;
                  //LOGE("Frame: %i, cc distance: %f", i, distance);
                }

                glm::vec4 trans_pcd_point_ru = relativ_transformation * curr_pcd_point_ru;
                distance = GetEuclideanDistance(
                    glm::vec3(curr_pcd_point_ru[0],curr_pcd_point_ru[1],curr_pcd_point_ru[2]),
                    glm::vec3(trans_pcd_point_ru[0],trans_pcd_point_ru[1], trans_pcd_point_ru[2]));

                if (distance <= 0.1f) {
                  distance_accum += distance;
                  distance_accum_c++;
                  //LOGE("Frame: %i, ru distance: %f", i, distance);
                }

                glm::vec4 trans_pcd_point_rd = relativ_transformation * curr_pcd_point_rd;
                distance = GetEuclideanDistance(
                    glm::vec3(curr_pcd_point_rd[0],curr_pcd_point_rd[1],curr_pcd_point_rd[2]),
                    glm::vec3(trans_pcd_point_rd[0],trans_pcd_point_rd[1], trans_pcd_point_rd[2]));

                if (distance <= 0.1f) {
                  distance_accum += distance;
                  distance_accum_c++;
                  //LOGE("Frame: %i, rd distance: %f", i, distance);
                }

                float distance_average = 0;
                if (distance_accum_c > 0) {
                  distance_average = distance_accum / distance_accum_c;
                  if (distance_average <= 0.6f) {
                    LOGE("Frame %i has a distance average of %f", i, distance_average);
                    count++;
                  }
                }

                /*glm::vec4 trans_pcd_point = relativ_transformation * curr_pcd_point;
                glm::vec3 tmp(trans_pcd_point[0], trans_pcd_point[1], trans_pcd_point[2]);
                // get pixel coordinates of trans_pcd_point
                //LOGE("trans_pcd: x %f, y %f, z %f", tmp[0], tmp[1], tmp[2]);
                glm::vec3 trans_depth_point = Convert3DPointToDepthPoint(tmp);
                //LOGE("trans_pixel: x %f, y %f", trans_depth_point[0], trans_depth_point[1]);
                // check if point is out of the image borders
                if (trans_depth_point[0] > 0 &&
                    trans_depth_point[0] < color_camera_intrinsics.width &&
                    trans_depth_point[1] > 0 &&
                    trans_depth_point[1] < color_camera_intrinsics.height) {
                  float distance = GetEuclideanDistance(curr_depth_point, trans_depth_point);
                  if (trans_pcd_point[2] > 2.0f || trans_pcd_point[2] < 0.05f) {
                    //LOGE("Frame: %i, distance: %f but z: %f not in frustum", i, distance, trans_pcd_point[2]);
                  } else if (distance > 150) {
                    //LOGE("Frame: %i, distance is to big: %f", i, distance);
                  } else {
                    LOGE("Frame: %i, distance: %f", i, distance);

                    // get icp pose from scan matcher
                    //LOGE("Get loop pose start...");
                    Eigen::Isometry3f loop_pose = scan_matcher_->Match(
                        point_cloud_container_->GetPointCloudContainer()[i]->GetXYZValues(),
                        point_cloud_container_->GetPointCloudContainer()[point_cloud_container_->GetLatestPCD()->GetId()]->GetXYZValues(),
                        slam_->GetPose(
                            point_cloud_container_->GetPointCloudContainer()[i]->GetId()),
                        slam_->GetPose(
                            point_cloud_container_->GetPointCloudContainer()[point_cloud_container_->GetLatestPCD()->GetId()]->GetId()));

                    Eigen::Isometry3d relative_pose = util::CastIsometry3fTo3d(loop_pose);
                    //LOGE("Get loop pose stop...");

                    //LOGE("Add loop closure...");
                    int confidence = 160 / distance + 1;
                    slam_->AddLoopClosure(
                        point_cloud_container_->GetPointCloudContainer()[i]->GetId(),
                        point_cloud_container_->GetPointCloudContainer()[point_cloud_container_->GetLatestPCD()->GetId()]->GetId(),
                        relative_pose, confidence);
                  }
                } else {
                  // isn't a potential loop closure frame
                  //LOGE("Frame: %i, distance: out of image bound", i);
                }*/
                //store_image_ = false;
              }
            }
          }

          if (store_point_clouds_) {

            for (int i = 0; i < point_cloud_container_->GetPointCloudContainer().size(); i++) {
              pcd_file_writer_->SetPCDRGBData(
                  point_cloud_container_->GetPointCloudContainer()[i]->GetPCDData(),
                  point_cloud_container_->GetPointCloudContainer()[i]->GetTranslation(),
                  point_cloud_container_->GetPointCloudContainer()[i]->GetRotation());
              pcd_file_writer_->SetUnordered();
              pcd_file_writer_->SaveToFile("Loop_banana", i);
            }
            LOGE("Found %i loop closures", count);
            store_image_ = false;

            /*for (int i = 0;  i < point_cloud_container_->GetPointCloudContainer().size(); i++) {
              Eigen::Isometry3f loop_pose = scan_matcher_->Match(
                  point_cloud_container_->GetPointCloudContainer()[i]->GetXYZValues(),
                  point_cloud_container_->GetPointCloudContainer()[point_cloud_container_->GetLatestPCD()->GetId()]->GetXYZValues(),
                  slam_->GetPose(
                      point_cloud_container_->GetPointCloudContainer()[i]->GetId()),
                  slam_->GetPose(
                      point_cloud_container_->GetPointCloudContainer()[point_cloud_container_->GetLatestPCD()->GetId()]->GetId()));

              Eigen::Isometry3d relative_pose = util::CastIsometry3fTo3d(loop_pose);
              //LOGE("Get loop pose stop...")

            }*/

            /*slam_->SaveGraph();

            LOGE("Optimize graph start...");
            slam_->OptimizeGraph();
            LOGE("optimize graph stop...");

            slam_->SaveGraph();

            std::vector <Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> all_poses = slam_->GetPoses();

            glm::mat4 relative_transformation_icp;

            for (int i = 0; i < all_poses.size(); i++) {
              glm::mat4 icppose_glm = util::ConvertEigenToGLMPose(all_poses[i]);
              glm::vec3 icp_translation = util::GetTranslationFromMatrix(icppose_glm);
              glm::quat icp_rotation = util::GetRotationFromMatrix(icppose_glm);

              //icp_positions[i] = icp_translation;
              point_cloud_container_->GetPointCloudContainer()[i]->SetTranslation(
                  icp_translation);
              point_cloud_container_->GetPointCloudContainer()[i]->SetRotation(icp_rotation);
            }

            for (int i = 0; i < point_cloud_container_->GetPointCloudContainer().size(); i++) {
              pcd_file_writer_->SetPCDRGBData(
                  point_cloud_container_->GetPointCloudContainer()[i]->GetPCDData(),
                  point_cloud_container_->GetPointCloudContainer()[i]->GetTranslation(),
                  point_cloud_container_->GetPointCloudContainer()[i]->GetRotation());
              pcd_file_writer_->SetUnordered();
              pcd_file_writer_->SaveToFile("Loop_book_opt", i+1);
            }*/
            store_point_clouds_ = false;
          }

          if (point_cloud_container_->GetLatestPCD() != nullptr) {
            scene_->Render(open_gl_T_cur_pose_T_gl_camera, glm::inverse(open_gl_T_cur_pose),
                           icppose_glm,
                           point_cloud_container_->GetLatestPCD()->GetXYZValues_T_Color(),
                           point_cloud_container_->GetLatestPCD()->GetRGBValues());
          }
        } else {
          LOGE("Invalid pose for ss_t_depth at time: %lf", depth_timestamp);
        }
      } else {
        LOGE("Invalid pose for ss_t_color at time: %lf", color_timestamp_);
      }
    }

    /*if (store_point_clouds_) {
      for (int i = 0; i < point_cloud_container_->GetPointCloudContainer().size(); i++) {
        pcd_file_writer_->SetPCDRGBData(
            point_cloud_container_->GetPointCloudContainer()[i]->GetPCDData(),
            point_cloud_container_->GetPointCloudContainer()[i]->GetTranslation(),
            point_cloud_container_->GetPointCloudContainer()[i]->GetRotation());
        pcd_file_writer_->SetUnordered();
        pcd_file_writer_->SaveToFile();
      }
      store_point_clouds_ = false;
    }*/
  }

  void SynchronizationApplication::OnTouchEvent(int touch_count,
                                   tango_gl::GestureCamera::TouchEvent event,
                                   float x0, float y0, float x1, float y1) {
    scene_->OnTouchEvent(touch_count, event, x0, y0, x1, y1);
  }

  void SynchronizationApplication::SetCameraType(tango_gl::GestureCamera::CameraType camera_type) {
    scene_->SetCameraType(camera_type);
  }

  void SynchronizationApplication::FreeGLContent() {
    delete color_image_;
    delete range_image_;
  }

  void SynchronizationApplication::SetDepthMap(bool on) {
    optimize_pose_graph_ = on;
  }

  void SynchronizationApplication::SetRGBMap(bool on) {
    render_range_image_ = on;
  }

  void SynchronizationApplication::SetSocket(std::string addr, int port) {
    socket_addr_ = addr;
    socket_port_ = port;
  }

  void SynchronizationApplication::SetStartPCDRecording(bool on) {
    update_point_cloud_container = on;
  }

  void SynchronizationApplication::SetSendPCDRecording(bool on) {
    send_point_cloud_container = on;
  }

  glm::mat4 SynchronizationApplication::GetExtrinsicsAppliedOpenGLWorldFrame(
      const glm::mat4 pose_matrix) {
    // This full multiplication is equal to:
    //   opengl_world_T_opengl_camera =
    //      opengl_world_T_start_service *
    //      start_service_T_device *
    //      device_T_imu *
    //      imu_T_depth_camera *
    //      depth_camera_T_opengl_camera;
    //
    // More information about frame transformation can be found here:
    // Frame of reference:
    //   https://developers.google.com/project-tango/overview/frames-of-reference
    // Coordinate System Conventions:
    //   https://developers.google.com/project-tango/overview/coordinate-systems
    return tango_gl::conversions::opengl_world_T_tango_world() * pose_matrix *
           device_T_depth_ *
           tango_gl::conversions::depth_camera_T_opengl_camera();
  }


}  // namespace rgb_depth_sync
