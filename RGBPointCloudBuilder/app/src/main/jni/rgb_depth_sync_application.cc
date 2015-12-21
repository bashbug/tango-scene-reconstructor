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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tango-gl/conversions.h>
#include <rgb-depth-sync/rgb_depth_sync_application.h>
#include "rgb-depth-sync/write_color_image.h"
#include "rgb-depth-sync/point_cloud_data.h"
#include <Eigen/Geometry>

namespace rgb_depth_sync {

  inline float deg2rad(float alpha) {
    return alpha*M_PI/180.0f;
  }

  inline void clamp(uint8_t* num) {
    if (*num < 0) {
      *num = 0;
    }
    if (*num > 255) {
      *num = 255;
    }
  }

  void OnFrameAvailableRouter(void* context, TangoCameraId,
                              const TangoImageBuffer* buffer) {
    SynchronizationApplication* app = static_cast<SynchronizationApplication*>(context);
    app->OnFrameAvailable(buffer);
  }

  void SynchronizationApplication::OnFrameAvailable(const TangoImageBuffer* buffer) {

    yuv_width_ = buffer->width;
    yuv_height_ = buffer->height;
    uv_buffer_offset_ = yuv_width_ * yuv_height_;
    yuv_size_ = yuv_width_ * yuv_height_ + yuv_width_ * yuv_height_ / 2;

    size_t yuv_size = buffer->width * buffer->height + buffer->width * buffer->height / 2;

    callback_yuv_buffer_.resize(yuv_size_);
    render_yuv_buffer_.resize(yuv_size_);

    memcpy(&callback_yuv_buffer_[0], buffer->data, yuv_size_);
    {
      std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
      color_timestamp_ = buffer->timestamp;
      callback_yuv_buffer_.swap(shared_yuv_buffer_);
      swap_yuv_buffer_signal_ = true;
    }

  }

  void OnPoseAvailableRouter(void* context, const TangoPoseData* pose) {
      SynchronizationApplication* app =
          static_cast<SynchronizationApplication*>(context);
      app->OnPoseAvailable(pose);
  }

  void SynchronizationApplication::OnPoseAvailable(const TangoPoseData* pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    // Store only valid poses
    if(pose->status_code == TANGO_POSE_VALID) {
      //pose_container_->UpdateContainer(pose);
    }
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
      callback_point_cloud_buffer_.swap(shared_point_cloud_buffer_);
      swap_point_cloud_buffer_signal_ = true;
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
    return TangoService_connectTextureId(TANGO_CAMERA_COLOR, color_image_->GetTextureId(),
                                         this, nullptr);
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

    TangoCameraIntrinsics color_camera_intrinsics;
    TangoErrorType ret = TangoService_getCameraIntrinsics(
        TANGO_CAMERA_COLOR, &color_camera_intrinsics);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to get the intrinsics for the color"
          "camera.");
      return ret;
    }

    range_image_->SetCameraIntrinsics(color_camera_intrinsics);
    pcd_->SetCameraIntrinsics(color_camera_intrinsics);
    point_cloud_container_->SetCameraIntrinsics(color_camera_intrinsics);

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
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_device);
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

    LOGE("imu_T_depth: %f, %f, %f, %f", imu_T_depth[0][0], imu_T_depth[1][0], imu_T_depth[2][0], imu_T_depth[3][0]);
    LOGE("imu_T_depth: %f, %f, %f, %f", imu_T_depth[0][1], imu_T_depth[1][1], imu_T_depth[2][1], imu_T_depth[3][1]);
    LOGE("imu_T_depth: %f, %f, %f, %f", imu_T_depth[0][2], imu_T_depth[1][2], imu_T_depth[2][2], imu_T_depth[3][2]);
    LOGE("imu_T_depth: %f, %f, %f, %f", imu_T_depth[0][3], imu_T_depth[1][3], imu_T_depth[2][3], imu_T_depth[3][3]);

    glm::mat4 device_T_imu = glm::inverse(imu_T_device);

    device_T_color_ = glm::inverse(imu_T_device) * imu_T_color;
    device_T_depth_ = glm::inverse(imu_T_device) * imu_T_depth;
    color_T_device_ = glm::inverse(device_T_color_);
    glm::mat4 color_T_depth = color_T_device_ * device_T_depth_;

    LOGE("color_T_depth: %f, %f, %f, %f", color_T_depth[0][0], color_T_depth[1][0], color_T_depth[2][0], color_T_depth[3][0]);
    LOGE("color_T_depth: %f, %f, %f, %f", color_T_depth[0][1], color_T_depth[1][1], color_T_depth[2][1], color_T_depth[3][1]);
    LOGE("color_T_depth: %f, %f, %f, %f", color_T_depth[0][2], color_T_depth[1][2], color_T_depth[2][2], color_T_depth[3][2]);
    LOGE("color_T_depth: %f, %f, %f, %f", color_T_depth[0][3], color_T_depth[1][3], color_T_depth[2][3], color_T_depth[3][3]);

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

  void SynchronizationApplication::InitializeGLContent() {

    color_image_ = new rgb_depth_sync::ColorImage();
    range_image_ = new rgb_depth_sync::RangeImage();
    pcd_ = new rgb_depth_sync::PointCloudData();
    point_cloud_container_ = new rgb_depth_sync::PointCloudContainer();
    pose_container_ = new rgb_depth_sync::PoseContainer();
    scene_ = new rgb_depth_sync::Scene();

    float angluarResolutionX = deg2rad(0.5f);
    float angularResolutionY = angluarResolutionX;
    projective_scan_matcher_ = new ProjectiveScanMatcher3d();

    // set inital parameters of the scan matcher which is used for estimate the pose of the frames of loop closures
    projective_scan_matcher_->parameters.searchRadius = 2;
    projective_scan_matcher_->parameters.sourceImageStartStepSizeX = projective_scan_matcher_->parameters.sourceImageStartStepSizeY = 3;
    projective_scan_matcher_->parameters.maxDistanceStart = 0.5f;
    projective_scan_matcher_->parameters.maxDistanceEnd = 0.02f;
    projective_scan_matcher_->parameters.minScanOverlap = 1.0f;
    projective_scan_matcher_->parameters.numIterations = 100;
    projective_scan_matcher_->parameters.maxDistanceBetweenScans = 0.5;

    Eigen::Isometry3f sensorPose = Eigen::Isometry3f::Identity();
    projective_image_ = new SphericalProjectiveImage(angluarResolutionX, angularResolutionY,
                                                              sensorPose, ProjectiveImage::CAMERA_FRAME);
    //projective_image_->useAutomaticResize(true);
    projective_image_->resizeImage(128, 77, 296, 143);
    projective_image_->sortPixelPointsRegardingRange();

    icpPose_ = Eigen::Isometry3f::Identity();
  }

  void SynchronizationApplication::Render() {
    double rgb_timestamp = 0.0;
    double color_timestamp = 0.0;
    double depth_timestamp = 0.0;
    bool new_points = false;
    bool new_rgb_values = false;

    //render_point_cloud_buffer_.clear();
    //render_yuv_buffer_.clear();

    {
      std::lock_guard<std::mutex> lock(point_cloud_mutex_);
      depth_timestamp = depth_timestamp_;
      if (swap_point_cloud_buffer_signal_) {
        shared_point_cloud_buffer_.swap(render_point_cloud_buffer_);
        swap_point_cloud_buffer_signal_ = false;
        new_points = true;
      }
    }

    {
      std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
      color_timestamp = color_timestamp_;
      if (swap_yuv_buffer_signal_) {
        shared_yuv_buffer_.swap(render_yuv_buffer_);
        swap_yuv_buffer_signal_ = false;
        new_rgb_values = true;
      }
    }

    // We need to make sure that we update the texture associated with the color
    // image.
    if (TangoService_updateTexture(TANGO_CAMERA_COLOR, &color_timestamp) !=
        TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to get a color image.");
    }

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
    if (TangoService_getPoseAtTime(rgb_timestamp, color_frame_pair,
                                   &pose_start_service_T_device_t1) !=
        TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Could not find a valid pose at time %lf"
              " for the color camera.",
          color_timestamp);
    }

    glm::mat4 start_service_T_device_t0 = util::GetMatrixFromPose(&pose_start_service_T_device_t0);
    glm::mat4 start_service_T_device_t1 = util::GetMatrixFromPose(&pose_start_service_T_device_t1);
    glm::mat4 device_t0_T_depth_t0 = device_T_depth_;
    glm::mat4 color_t1_T_device_t1 = glm::inverse(device_T_color_);
    glm::mat4 start_service_T_color_t1 = start_service_T_device_t1 * device_T_color_;

    SetRGBBuffer();

    if(store_image_){
      StoreImage();
    }

    if (pose_start_service_T_device_t1.status_code == TANGO_POSE_VALID) {
      if (pose_start_service_T_device_t0.status_code == TANGO_POSE_VALID) {

        glm::mat4 color_T_depth =
            color_t1_T_device_t1 *
            glm::inverse(start_service_T_device_t1) *
            start_service_T_device_t0 *
            device_t0_T_depth_t0;

        if(render_depth_map_) {
          range_image_->RenderDepthMap(color_T_depth,
                                       start_service_T_color_t1,
                                       render_point_cloud_buffer_,
                                       rgb_map_buffer_,
                                       rgb_pcd_buffer_,
                                       color_timestamp);
          render_image_ = false;
        } else if(render_range_image_) {
          range_image_->RenderRGBMap(color_T_depth,
                                     start_service_T_color_t1,
                                     render_point_cloud_buffer_,
                                     rgb_map_buffer_,
                                     rgb_pcd_buffer_,
                                     color_timestamp);
          render_image_ = false;
        } else {

          if (render_point_cloud_buffer_.size() > 0 && rgb_map_buffer_.size() > 0) {
            // clear pixels befor adding new ones
            projective_image_->clearPixels();
            for (size_t i = 0; i < render_point_cloud_buffer_.size(); i += 3) {
              // adding new depth data to the projective_image_
              projective_image_->addPoint(&render_point_cloud_buffer_[i], i / 3);
              if (std::abs(render_point_cloud_buffer_[i]) < 1e-4 &&
                  std::abs(render_point_cloud_buffer_[i + 1]) < 1e-4 &&
                  std::abs(render_point_cloud_buffer_[i + 2]) < 1e-4) {
                LOGE("x, y, z == 0");
              }
            }
            // sorting pixels
            projective_image_->sortPixelPointsRegardingRange();

            // get tango pose at time of the given color data
            Eigen::Isometry3f odometryPose = convertGLMToEigenPose(start_service_T_color_t1);

            Eigen::Isometry3f initalGuess = icpPose_;
            static bool first_image = true;
            static Eigen::Isometry3f lastOdometryPose = odometryPose;

            if (first_image) {
              initalGuess = odometryPose;
            } else {
              initalGuess = icpPose_ * lastOdometryPose.inverse() * odometryPose;
              first_image = false;
            }

            lastOdometryPose = odometryPose;

            // get icp pose for given depth data
            icpPose_ = projective_scan_matcher_->matchNewScan(*projective_image_, &initalGuess);

            glm::mat4 open_gl_T_cur_pose_T_gl_camera =
                tango_gl::conversions::opengl_world_T_tango_world() *
                start_service_T_color_t1 * tango_gl::conversions::depth_camera_T_opengl_camera();

            glm::mat4 open_gl_T_cur_pose =
                tango_gl::conversions::opengl_world_T_tango_world() *
                start_service_T_color_t1;

            glm::mat4 icppose_glm = convertEigenToGLMPose(icpPose_);

            icppose_glm =
                tango_gl::conversions::opengl_worl_T_icp_world() *
                icppose_glm * tango_gl::conversions::depth_camera_T_opengl_camera();

            pcd_->SetRGBDData(color_T_depth,
                              open_gl_T_cur_pose,
                              render_point_cloud_buffer_,
                              rgb_map_buffer_,
                              rgb_pcd_buffer_);

            scene_->Render(open_gl_T_cur_pose_T_gl_camera, open_gl_T_cur_pose, icppose_glm,
                           pcd_->GetVertices(),
                           pcd_->GetRGBValues());
          }

          if (store_point_clouds_ || send_point_clouds_) {
            /*Eigen::Quaternionf eigen_rot(icpPose_.rotation());
            glm::vec3 translation = glm::vec3(icpPose_.translation().x(), icpPose_.translation().y(),
                                              icpPose_.translation().z());
            glm::quat rotation = glm::quat(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(),
                                           eigen_rot.z());*/

            glm::vec3 translation = util::GetTranslationFromMatrix(start_service_T_color_t1);
            glm::quat rotation = util::GetRotationFromMatrix(start_service_T_color_t1);

            pcd_->setPCDData(pcd_->GetRGBDData(), translation, rotation,
                             color_timestamp);
            pcd_->setUnordered();

            if(store_point_clouds_) {
              pcd_->saveToFile();
            }
            if(send_point_clouds_) {
              pcd_->saveToSocket(socket_addr_, socket_port_);
            }
          }

        }
      } else {
        LOGE("Invalid pose for ss_t_depth at time: %lf", depth_timestamp);
      }
    } else {
      LOGE("Invalid pose for ss_t_color at time: %lf", color_timestamp);
    }
    if (render_depth_map_) {
      range_image_->Draw(screen_width_, screen_height_);
    }
    if(render_range_image_){
      range_image_->Draw(screen_width_, screen_height_);
    }
    if(render_image_) {
      //color_image_->Draw(screen_width_, screen_height_);
    }

    if (update_point_cloud_container) {
      //LOGE("update pcd");
      //point_cloud_container_->SavePointCloud(color_timestamp, range_image_->GetRGBPointCloud());
    }

    if (send_point_cloud_container) {
      point_cloud_container_->UpdateMergedPointCloud();
      glm::vec3 translation = glm::vec3(0.0f);
      glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
      pcd_->setPCDData(point_cloud_container_->GetMergedPointCloud(), translation, rotation, color_timestamp);
      pcd_->setUnordered();
      pcd_->saveToSocket(socket_addr_, socket_port_);
      send_point_cloud_container = false;
    }
  }

  void SynchronizationApplication::SetRGBBuffer() {

    rgb_map_buffer_;
    rgb_pcd_buffer_;
    rgb_map_buffer_.resize(yuv_width_ * yuv_height_ * 3);
    rgb_pcd_buffer_.resize(yuv_width_ * yuv_height_);

    size_t rgb_image_index = 0;

    rgb_map_buffer_.resize(yuv_width_ * yuv_height_ * 3);
    rgb_pcd_buffer_.resize(yuv_width_ * yuv_height_);

    for (size_t i = 0; i < yuv_height_; i++) {
      for (size_t j = 0; j < yuv_width_; j++) {
        size_t x_index = j;
        if (j % 2 != 0) {
          x_index = j - 1;
        }

        size_t rgb_index = (i * yuv_width_ + j) * 3;

        // The YUV texture format is NV21,
        // yuv_buffer_ buffer layout:
        //   [y0, y1, y2, ..., yn, v0, u0, v1, u1, ..., v(n/4), u(n/4)]
        Yuv2Rgb(render_yuv_buffer_[i * yuv_width_ + j],
                render_yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index + 1],
                render_yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index],
                &rgb_map_buffer_[rgb_index],
                &rgb_map_buffer_[rgb_index + 1],
                &rgb_map_buffer_[rgb_index + 2],
                &rgb_pcd_buffer_[i * yuv_width_ + j]);
      }
    }
  }

  glm::mat4 SynchronizationApplication::convertEigenToGLMPose(Eigen::Isometry3f eigen_pose) {
    glm::mat4 pose;
    //Rotation
    pose[0][0] = eigen_pose(0,0);
    pose[0][1] = eigen_pose(1,0);
    pose[0][2] = eigen_pose(2,0);
    pose[0][3] = eigen_pose(3,0);

    pose[1][0] = eigen_pose(0,1);
    pose[1][1] = eigen_pose(1,1);
    pose[1][2] = eigen_pose(2,1);
    pose[1][3] = eigen_pose(3,1);

    pose[2][0] = eigen_pose(0,2);
    pose[2][1] = eigen_pose(1,2);
    pose[2][2] = eigen_pose(2,2);
    pose[2][3] = eigen_pose(3,2);

    //Translation
    pose[3][0] = eigen_pose(0,3);
    pose[3][1] = eigen_pose(1,3);
    pose[3][2] = eigen_pose(2,3);
    pose[3][3] = eigen_pose(3,3);
    return pose;
  }

  Eigen::Isometry3f SynchronizationApplication::convertGLMToEigenPose(glm::mat4 glm_pose) {
    Eigen::Isometry3f pose;
    // Rotation
    pose(0,0) = glm_pose[0][0];
    pose(1,0) = glm_pose[0][1];
    pose(2,0) = glm_pose[0][2];
    pose(3,0) = glm_pose[0][3];

    pose(0,1) = glm_pose[1][0];
    pose(1,1) = glm_pose[1][1];
    pose(2,1) = glm_pose[1][2];
    pose(3,1) = glm_pose[1][3];

    pose(0,2) = glm_pose[2][0];
    pose(1,2) = glm_pose[2][1];
    pose(2,2) = glm_pose[2][2];
    pose(3,2) = glm_pose[2][3];

    // Translation
    pose(0,3) = glm_pose[3][0];
    pose(1,3) = glm_pose[3][1];
    pose(2,3) = glm_pose[3][2];
    pose(3,3) = glm_pose[3][3];
    return pose;
  }

  void SynchronizationApplication::StoreImage() {
    std::stringstream ss;

    current_timestamp_ = (int) std::ceil(color_timestamp_ * 1000);
    if (current_timestamp_ != previous_timestamp_) {
      ss << current_timestamp_;
      std::string filename =
          "/storage/emulated/0/Documents/RGBPointCloudBuilder/PPM/" + ss.str() + ".ppm";
      WriteColorImage *wci = new rgb_depth_sync::WriteColorImage(filename.c_str(), rgb_map_buffer_,
                                                                 yuv_width_, yuv_height_);
    }
    previous_timestamp_ = current_timestamp_;
    store_image_ = false;
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
    render_depth_map_ = on;
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

  // We could do this conversion in a fragment shader if all we care about is
  // rendering, but we show it here as an example of how people can use RGB data
  // on the CPU.
  void SynchronizationApplication::Yuv2Rgb(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t* r,
                                           uint8_t* g, uint8_t* b, uint32_t* rgb) {
    *r = yValue + (1.370705f * (vValue - 128.0f));
    *g = yValue - (0.698001f * (vValue - 128.0f)) - (0.337633f * (uValue - 128.0f));
    *b = yValue + (1.732446f * (uValue - 128.0f));

    clamp(&(*r));
    clamp(&(*g));
    clamp(&(*b));

    *rgb = ((uint32_t) (*r)) << 16 | ((uint32_t) (*g)) << 8 | (uint32_t) (*b);
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
