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

#include "rgb-depth-sync/rgb_depth_sync_application.h"

namespace rgb_depth_sync {

  void OnFrameAvailableRouter(void* context, TangoCameraId, const TangoImageBuffer* buffer) {
    SynchronizationApplication* app = static_cast<SynchronizationApplication*>(context);
    app->OnFrameAvailable(buffer);
  }

  void SynchronizationApplication::OnFrameAvailable(const TangoImageBuffer* buffer) {
    TangoPoseData ss_T_device_rgb_timestamp;
    TangoCoordinateFramePair color_frame_pair;
    color_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    color_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    if (TangoService_getPoseAtTime(buffer->timestamp, color_frame_pair,
                                   &ss_T_device_rgb_timestamp) !=
        TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Could not find a valid pose at time %lf"
              " for the color camera.",
          buffer->timestamp);
    } else {
      if(ss_T_device_rgb_timestamp.status_code == TANGO_POSE_VALID) {
        pcd_worker_->SetRGBBuffer(buffer);
      } else {
        LOGE("ss_T_device_rgb_timestamp pose not valid");
      }
    }
  }

  void OnXYZijAvailableRouter(void* context, const TangoXYZij* xyz_ij) {
    SynchronizationApplication* app = static_cast<SynchronizationApplication*>(context);
    app->OnXYZijAvailable(xyz_ij);
  }

  void SynchronizationApplication::OnXYZijAvailable(const TangoXYZij* xyz_ij) {
    TangoPoseData ss_T_device_xyz_timestamp;
    TangoCoordinateFramePair depth_frame_pair;
    depth_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    depth_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    if (TangoService_getPoseAtTime(xyz_ij->timestamp, depth_frame_pair,
                                   &ss_T_device_xyz_timestamp) !=
        TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Could not find a valid pose at time %lf"
              " for the depth camera.",
          xyz_ij->timestamp);
    } else {
      if (ss_T_device_xyz_timestamp.status_code == TANGO_POSE_VALID) {
        pcd_worker_->SetXYZBuffer(xyz_ij);
      } else {
        LOGE("ss_T_device_xyz_timestamp pose not valid");
      }
    }
  }

  SynchronizationApplication::SynchronizationApplication() {
    std::mutex* pcd_mtx;
    std::condition_variable* consume_pcd;

    pcd_container_ = new rgb_depth_sync::PCDContainer(pcd_mtx, consume_pcd);
    pcd_worker_ = new rgb_depth_sync::PCDWorker(pcd_container_);

    std::thread pcd_worker_thread(&rgb_depth_sync::PCDWorker::OnPCDAvailable, pcd_worker_);
    pcd_worker_thread.detach();
    slam_ = new rgb_depth_sync::Slam3D(pcd_container_, pcd_mtx, consume_pcd);

  }

  SynchronizationApplication::~SynchronizationApplication() {
    LOGE("Destroy SynchronizationApplication");
  }

  int SynchronizationApplication::TangoInitialize(JNIEnv* env, jobject caller_activity) {
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

    // Enable color camera from config.
    ret = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
    if (ret != TANGO_SUCCESS) {
      LOGE("Failed to enable cplor.");
      return ret;
    }

    // Note that it's super important for AR applications that we enable low
    // latency imu integration so that we have pose information available as
    // quickly as possible.
    ret = TangoConfig_setBool(tango_config_, "config_enable_low_latency_imu_integration", true);
    if (ret != TANGO_SUCCESS) {
      LOGE("Failed to enable low latency imu integration.");
      return ret;
    }
    return ret;
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
    LOGE( "SynchronizationApplication: Failed to get the intrinsics for the color"
              "camera.");
      return ret;
    }

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
    scene_ = new rgb_depth_sync::Scene();
  }

  void SynchronizationApplication::Render() {
      icp_ = glm::mat4();
      pcd_ = pcd_container_->GetLatestPCD();
      if (pcd_ != nullptr) {
        scene_->Render(pcd_->GetOpenGL_T_OpenGLCameraPose(),
                       pcd_->GetOpenGL_T_RGB(), icp_,
                       pcd_->GetXYZValues(),
                       pcd_->GetRGBValues());
      }
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
  }

  void SynchronizationApplication::SetDepthMap(bool on) {
  }

  void SynchronizationApplication::SetRGBMap(bool on) {
  }

  void SynchronizationApplication::SetSocket(std::string addr, int port) {
    socket_addr_ = addr;
    socket_port_ = port;
  }

  void SynchronizationApplication::SetStartPCDRecording(bool on) {
  }

  void SynchronizationApplication::SetSendPCDRecording(bool on) {
  }
}  // namespace rgb_depth_sync
