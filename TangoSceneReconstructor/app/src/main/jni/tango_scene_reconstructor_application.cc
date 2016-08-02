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

#include "tango-scene-reconstructor/tango_scene_reconstructor_application.h"

namespace tango_scene_reconstructor {

  void OnFrameAvailableRouter(void* context, TangoCameraId, const TangoImageBuffer* buffer) {
    TangoSceneReconstructorApplication* app = static_cast<TangoSceneReconstructorApplication*>(context);
    app->OnFrameAvailable(buffer);
  }

  void TangoSceneReconstructorApplication::OnFrameAvailable(const TangoImageBuffer* buffer) {
      if (!optimize_) {
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
            TangoSupport_updateImageBuffer(yuv_manager_, buffer);
          } else {
            LOGE("ss_T_device_rgb_timestamp pose not valid");
          }
        }
      }
  }

  void OnXYZijAvailableRouter(void* context, const TangoXYZij* xyz_ij) {
    TangoSceneReconstructorApplication* app = static_cast<TangoSceneReconstructorApplication*>(context);
    app->OnXYZijAvailable(xyz_ij);
  }

  void TangoSceneReconstructorApplication::OnXYZijAvailable(const TangoXYZij* xyz_ij) {
      if (!optimize_) {
        TangoPoseData ss_T_device_xyz_timestamp;
        TangoCoordinateFramePair depth_frame_pair;
        depth_frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
        depth_frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
        if (TangoService_getPoseAtTime(xyz_ij->timestamp, depth_frame_pair, &ss_T_device_xyz_timestamp) != TANGO_SUCCESS) {
          LOGE("SynchronizationApplication: Could not find a valid pose at time %lf for the depth camera.",
               xyz_ij->timestamp);
        } else {
          if (ss_T_device_xyz_timestamp.status_code == TANGO_POSE_VALID) {
            TangoSupport_updatePointCloud(xyz_manager_, xyz_ij);
            consume_xyz_->notify_one();
          } else {
            LOGE("ss_T_device_xyz_timestamp pose not valid");
          }
        }
      }
  }

  void OnPoseAvailableRouter(void* context, const TangoPoseData* pose) {
    TangoSceneReconstructorApplication* app = static_cast<TangoSceneReconstructorApplication*>(context);
    app->OnPoseAvailable(pose);
  }

  void TangoSceneReconstructorApplication::OnPoseAvailable(const TangoPoseData* pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_data_->UpdatePose(pose);
  }

  TangoSceneReconstructorApplication::TangoSceneReconstructorApplication() {
  }

  TangoSceneReconstructorApplication::~TangoSceneReconstructorApplication() {
    LOGE("Destroy SynchronizationApplication");
  }

  void TangoSceneReconstructorApplication::OnTangoServiceConnected(JNIEnv* env, jobject binder) {
    TangoErrorType ret = TangoService_setBinder(env, binder);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "SynchronizationApplication: Failed to set Tango service binder with"
          "error code: %d",
          ret);
    }
  }

  int TangoSceneReconstructorApplication::TangoInitialize(JNIEnv* env, jobject caller_activity, JavaVM* javaVM) {

    env_ = env;
    caller_activity_ = reinterpret_cast<jobject>(env->NewGlobalRef(caller_activity));
    javaVM_ = javaVM;
    jclass cls = env_->GetObjectClass(caller_activity_);
    activity_class_ = (jclass) env_->NewGlobalRef(cls);

    pose_optimization_id_ = -1;
    optimizationMethods_ = -1;

    xyz_mtx_ = std::make_shared<std::mutex>();
    consume_xyz_ = std::make_shared<std::condition_variable>();

    point_cloud_manager_ = new tango_scene_reconstructor::PointCloudManager(xyz_mtx_, consume_xyz_);
    std::thread point_cloud_manager_thread(&tango_scene_reconstructor::PointCloudManager::OnPCDAvailable, point_cloud_manager_);
    point_cloud_manager_thread.detach();
    point_cloud_manager_->Start();
    pose_data_ = PoseData::GetInstance();

    return 1;
  }

  int TangoSceneReconstructorApplication::TangoSetupConfig() {
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
      LOGE("Failed to enable color.");
      return ret;
    }

    // Set auto-recovery for motion tracking as requested by the user.
    ret = TangoConfig_setBool(tango_config_, "config_enable_auto_recovery", true);
    if (ret != TANGO_SUCCESS) {
      LOGE("PointCloudApp: config_enable_auto_recovery() failed with error"
               "code: %d", ret);
      return ret;
    }

    int32_t max_point_cloud_elements;
    ret = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements",
                                   &max_point_cloud_elements);

    LOGE("MAX POINT CLOUD ELEMENTS %i", max_point_cloud_elements);

    if(ret != TANGO_SUCCESS) {
      LOGE("Failed to query maximum number of point cloud elements.");
      return ret;
    }

    TangoSupport_createPointCloudManager(max_point_cloud_elements, &xyz_manager_);
    TangoSupport_createImageBufferManager(TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP, 1280, 720, &yuv_manager_);

    point_cloud_manager_->SetManagers(xyz_manager_, yuv_manager_);

    return ret;
  }

  int TangoSceneReconstructorApplication::TangoConnectCallbacks() {

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

    // Setting up the frame pair for the onPoseAvailable callback.
    TangoCoordinateFramePair pairs;
    pairs.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    pairs.target = TANGO_COORDINATE_FRAME_DEVICE;

    // Attach the onPoseAvailable callback.
    // The callback will be called after the service is connected.
    ret = TangoService_connectOnPoseAvailable(1, &pairs, OnPoseAvailableRouter);
    if (ret != TANGO_SUCCESS) {
      LOGE("PointCloudApp: Failed to connect to pose callback with error"
               "code: %d", ret);
      return ret;
    }
  }

  int TangoSceneReconstructorApplication::TangoConnect() {
    TangoErrorType ret = TangoService_connect(this, tango_config_);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to connect to the Tango service.");
    }
    return ret;
  }

  int TangoSceneReconstructorApplication::TangoSetIntrinsicsAndExtrinsics() {

    TangoErrorType ret;
    TangoPoseData pose_data;
    TangoCoordinateFramePair frame_pair;

    TangoPoseData pose_imu_T_device;
    TangoPoseData pose_imu_T_color;
    TangoPoseData pose_imu_T_depth;

    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_device);

    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
    TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_color);

    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
    TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_depth);

    glm::mat4 imu_T_device = util::GetMatrixFromPose(&pose_imu_T_device);
    glm::mat4 imu_T_color = util::GetMatrixFromPose(&pose_imu_T_color);
    glm::mat4 imu_T_depth = util::GetMatrixFromPose(&pose_imu_T_depth);

    glm::mat4 device_T_color = glm::inverse(imu_T_device) * imu_T_color;
    glm::mat4 device_T_depth = glm::inverse(imu_T_device) * imu_T_depth;
    glm::mat4 color_T_device = glm::inverse(device_T_color);

    pose_data_->SetImuTDevice(imu_T_device);
    pose_data_->SetImuTDepthCamera(imu_T_depth);
    pose_data_->SetImuTColorCamera(imu_T_color);
    pose_data_->SetDeviceTColorCamera(device_T_color);
    pose_data_->SetDeviceTDepthCamera(device_T_depth);
    pose_data_->SetColorCameraTDevice(color_T_device);

    TangoCameraIntrinsics depth_camera_intrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_DEPTH, &depth_camera_intrinsics);
    TangoCameraIntrinsics color_camera_intrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &color_camera_intrinsics);

    pose_data_->SetColorCameraIntrinsics(color_camera_intrinsics);
    pose_data_->SetDepthCameraIntrinsics(depth_camera_intrinsics);

    point_cloud_manager_->tango_mesh_reconstructor_->SetColorCamera3DRIntrinsics();

    return ret;
  }

  void TangoSceneReconstructorApplication::TangoDisconnect() {
    TangoConfig_free(tango_config_);
    tango_config_ = nullptr;
    xyz_manager_ = nullptr;
    yuv_manager_ = nullptr;
    TangoService_disconnect();
  }

  void TangoSceneReconstructorApplication::SetViewPort(int width, int height) {
    scene_->SetViewPort(width, height);
  }

  void TangoSceneReconstructorApplication::InitializeGLContent() {
    scene_ = new tango_scene_reconstructor::Scene();
  }

  void TangoSceneReconstructorApplication::Render() {
    if (!optimize_) {
      curr_pose_ = pose_data_->GetLatestPoseMatrix();
      xyz_buffer_.clear();
      rgb_buffer_.clear();
      xyz_buffer_ = point_cloud_manager_->point_cloud_reconstructor_->GetXYZValues(curr_pose_);
      rgb_buffer_ = point_cloud_manager_->point_cloud_reconstructor_->GetRGBValues();
      centroid_matrix_ = point_cloud_manager_->GetCentroidMatrix();
      LOGE("XYZ size: %i", xyz_buffer_.size());
      LOGE("RGB size: %i", rgb_buffer_.size());
      scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_),
                     pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_),
                     xyz_buffer_,
                     rgb_buffer_);
      /*glm::mat4 pose = tango_gl::conversions::opengl_world_T_tango_world() *
                       pose_data_->GetLatestPoseMatrix() * pose_data_->GetDeviceTColorCamera() *
                       tango_gl::conversions::color_camera_T_opengl_camera();
      //scene_->Render(pose);
      xyz_buffer_.clear();
      xyz_buffer_ = point_cloud_manager_->tango_mesh_reconstructor_->GetXYZ();
      rgb_buffer_.clear();
      rgb_buffer_ = point_cloud_manager_->tango_mesh_reconstructor_->GetRGB();
      std::vector<unsigned int> indices = point_cloud_manager_->tango_mesh_reconstructor_->GetIndices();
      /*point_cloud_manager_->tango_mesh_reconstructor_->Render(scene_->GetGestureCamera()->GetProjectionMatrix(),
                                                              scene_->GetGestureCamera()->GetViewMatrix(),
                                                              pose);
      scene_->Render(pose, xyz_buffer_, indices, rgb_buffer_);*/
    } else {
      switch(pose_optimization_id_) {
        case 2:
          xyz_buffer_.clear();
          xyz_buffer_ = point_cloud_manager_->GetXYZValuesOptWithMSM(glm::inverse(curr_pose_*centroid_matrix_));
          rgb_buffer_.clear();
          rgb_buffer_ = point_cloud_manager_->GetRGBOptWithMSMValues();
          scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
          break;
        case 1:
          xyz_buffer_.clear();
          xyz_buffer_ = point_cloud_manager_->GetXYZValuesOptWithSM(glm::inverse(curr_pose_*centroid_matrix_));
          rgb_buffer_.clear();
          rgb_buffer_ = point_cloud_manager_->GetRGBOptWithSMValues();
          scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
          break;
        case 0:
          xyz_buffer_.clear();
          xyz_buffer_ = point_cloud_manager_->GetXYZValues(glm::inverse(curr_pose_*centroid_matrix_));
          rgb_buffer_.clear();
          rgb_buffer_ = point_cloud_manager_->GetRGBValues();
          scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
          break;
        default:
          scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
      }
    }
  }

  void TangoSceneReconstructorApplication::OptimizeAndSaveToFolder(std::string folder_name) {
    StopPCDWorker();

    optimize_ = true;

    FrameToFrameScanMatcher ftfsm;
    MultiframeScanMatcher mfsm;

    switch (optimizationMethods_) {
      case 0:
        ftfsm.Init(point_cloud_manager_);
        ftfsm.Optimize();
        break;
      case 1:
        mfsm.Init(point_cloud_manager_);
        mfsm.Optimize();
        break;
      case 2:
        ftfsm.Init(point_cloud_manager_);
        ftfsm.Optimize();
        mfsm.Init(point_cloud_manager_);
        mfsm.Optimize();
        break;
      default:
        break;
    }

    PCD* PCD = new tango_scene_reconstructor::PCD(optimizationMethods_);
    PCD->SavePointCloudContainer(point_cloud_manager_, folder_name);


    point_cloud_manager_->OptimizeMesh();

    /*pcl::io::savePCDFile (folder_name + "Mesh/FTFSM.pcd", *point_cloud_manager_->GetFTFSMMeshPCDFile());
    pcl::io::savePCDFile (folder_name + "Mesh/MFSM.pcd", *point_cloud_manager_->GetMFSMMeshPCDFile());*/

    jmethodID method = env_->GetMethodID(activity_class_, "setComputationTimes", "(IIII)V");
    env_->CallVoidMethod(caller_activity_, method,
                         reinterpret_cast<jint>(ftfsm.GetAverageComputationTime()),
                         reinterpret_cast<jint>(ftfsm.GetComputationTime()),
                         reinterpret_cast<jint>(mfsm.GetAverageComputationTime()),
                         reinterpret_cast<jint>(mfsm.GetComputationTime()));

    method = env_->GetMethodID(activity_class_, "setFTFInfo", "(III)V");
    env_->CallVoidMethod(caller_activity_, method,
                         reinterpret_cast<jint>(ftfsm.GetNoOfLoopClosures()),
                         reinterpret_cast<jint>(ftfsm.GetNoOfMatchedFrames()),
                         reinterpret_cast<jint>((int)point_cloud_manager_->point_cloud_container_.size()));

  }

  void TangoSceneReconstructorApplication::UpdatePCDs() {

    int lastIndex = point_cloud_manager_->GetPCDContainerLastIndex();
    for (int i = 0; i <= lastIndex; i++) {
      point_cloud_manager_->point_cloud_container_[i]->Update();
    }

  }

  void TangoSceneReconstructorApplication::StopPCDWorker() {
    point_cloud_manager_->Stop();
    while(point_cloud_manager_->IsRunning()) {
      // wait until all processes finished
    }
  }

  void TangoSceneReconstructorApplication::StartPCDWorker() {
    optimize_ = false;
    point_cloud_manager_->Stop();
    while(point_cloud_manager_->IsRunning()) {
      // wait until all processes finished
    }
    point_cloud_manager_->ResetPCD();
    point_cloud_manager_->Start();
  }

  void TangoSceneReconstructorApplication::OnTouchEvent(int touch_count,
                                                tango_gl::GestureCamera::TouchEvent event,
                                                float x0, float y0, float x1, float y1) {
    scene_->OnTouchEvent(touch_count, event, x0, y0, x1, y1);
  }

  void TangoSceneReconstructorApplication::SetCameraType(tango_gl::GestureCamera::CameraType camera_type) {
    scene_->SetCameraType(camera_type);
  }

  void TangoSceneReconstructorApplication::FreeGLContent() {
  }

  void TangoSceneReconstructorApplication::SetRangeValue(float range) {
    point_cloud_manager_->SetRangeValue(range);
  }

  void TangoSceneReconstructorApplication::ShowUnOPTMesh() {
    pose_optimization_id_ = 0;
  }

  void TangoSceneReconstructorApplication::ShowSMMesh() {
    pose_optimization_id_ = 1;
  }

  void TangoSceneReconstructorApplication::ShowMSMMesh() {
    pose_optimization_id_ = 2;
  }

  void TangoSceneReconstructorApplication::SetBackgroundColorBlack(bool on) {
    scene_->SetBackgroundColor(on);
  }

  void TangoSceneReconstructorApplication::SetGridOn(bool on) {
    scene_->SetGridOn(on);
  }

  void TangoSceneReconstructorApplication::SetOptimizationMethods(int opt) {
    optimizationMethods_ = opt;
  }

}  // namespace rgb_depth_sync
