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
            img_count_++;
          } else {
            LOGE("ss_T_device_rgb_timestamp pose not valid");
          }
        }
      }
  }

  void OnXYZijAvailableRouter(void* context, const TangoXYZij* xyz_ij) {
    SynchronizationApplication* app = static_cast<SynchronizationApplication*>(context);
    app->OnXYZijAvailable(xyz_ij);
  }

  void SynchronizationApplication::OnXYZijAvailable(const TangoXYZij* xyz_ij) {
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
            //std::unique_lock<std::mutex> lock(*xyz_mtx_);
            TangoSupport_updatePointCloud(xyz_manager_, xyz_ij);
            consume_xyz_->notify_one();
            pcd_count_++;
          } else {
            LOGE("ss_T_device_xyz_timestamp pose not valid");
          }
        }
      }
  }

  void OnPoseAvailableRouter(void* context, const TangoPoseData* pose) {
    SynchronizationApplication* app = static_cast<SynchronizationApplication*>(context);
    app->OnPoseAvailable(pose);
  }

  void SynchronizationApplication::OnPoseAvailable(const TangoPoseData* pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_data_->UpdatePose(pose);
  }

  SynchronizationApplication::SynchronizationApplication() {
  }

  SynchronizationApplication::~SynchronizationApplication() {
    LOGE("Destroy SynchronizationApplication");
  }

    void SynchronizationApplication::OnTangoServiceConnected(JNIEnv* env, jobject binder) {
      TangoErrorType ret = TangoService_setBinder(env, binder);
      if (ret != TANGO_SUCCESS) {
        LOGE(
            "SynchronizationApplication: Failed to set Tango service binder with"
            "error code: %d",
            ret);
      }
    }

  int SynchronizationApplication::TangoInitialize(JNIEnv* env, jobject caller_activity, JavaVM* javaVM) {

    env_ = env;
    caller_activity_ = reinterpret_cast<jobject>(env->NewGlobalRef(caller_activity));
    javaVM_ = javaVM;
    jclass cls = env_->GetObjectClass(caller_activity_);
    activity_class_ = (jclass) env_->NewGlobalRef(cls);

    show_sm_mesh_ = false;
    show_msm_mesh_ = false;
    show_unopt_mesh_ = false;

    pose_optimization_id_ = -1;

    pcd_mtx_ = std::make_shared<std::mutex>();
    consume_pcd_ = std::make_shared<std::condition_variable>();
    xyz_mtx_ = std::make_shared<std::mutex>();
    consume_xyz_ = std::make_shared<std::condition_variable>();

    curr_index_ = -1;
    prev_index_ = -1;
    first_index_ = true;

    save_pcd_ = false;
    start_pcd_ = false;
    pcd_count_ = 0;
    img_count_ = 0;
    pcd_container_optimized_ = false;
    pcd_container_optimized_mf_ = false;

    pcd_container_ = new rgb_depth_sync::PCDContainer();

    pcd_worker_ = new rgb_depth_sync::PCDWorker(xyz_mtx_, consume_xyz_, pcd_container_);
    std::thread pcd_worker_thread(&rgb_depth_sync::PCDWorker::OnPCDAvailable, pcd_worker_);
    pcd_worker_thread.detach();
    pcd_worker_->Start();

    pose_data_ = PoseData::GetInstance();

    optimizationMethods_ = -1;

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

    pcd_worker_->SetManagers(xyz_manager_, yuv_manager_);

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

  int SynchronizationApplication::TangoConnect() {
    TangoErrorType ret = TangoService_connect(this, tango_config_);
    if (ret != TANGO_SUCCESS) {
      LOGE("SynchronizationApplication: Failed to connect to the Tango service.");
    }
    return ret;
  }

  int SynchronizationApplication::TangoSetIntrinsicsAndExtrinsics() {

    TangoErrorType ret;
    TangoPoseData pose_data;
    TangoCoordinateFramePair frame_pair;

    // Get device with respect to imu transformation matrix.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "PointCloudApp: Failed to get transform between the IMU frame and "
              "device frames");
      return ret;
    }
    pose_data_->SetImuTDevice(pose_data_->GetMatrixFromPose(pose_data));

    // Get color camera with respect to imu transformation matrix.
    frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
    frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
    ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
    if (ret != TANGO_SUCCESS) {
      LOGE(
          "PointCloudApp: Failed to get transform between the color camera frame "
              "and device frames");
      return ret;
    }

    pose_data_->SetImuTDepthCamera(pose_data_->GetMatrixFromPose(pose_data));

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

    return ret;
  }

  void SynchronizationApplication::TangoDisconnect() {
    TangoConfig_free(tango_config_);
    tango_config_ = nullptr;
    xyz_manager_ = nullptr;
    yuv_manager_ = nullptr;
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
    if (!optimize_) {
      curr_pose_ = pose_data_->GetLatestPoseMatrix();
      xyz_buffer_.clear();
      xyz_buffer_ = pcd_container_->GetXYZValues(glm::inverse(curr_pose_));
      rgb_buffer_.clear();
      rgb_buffer_ = pcd_container_->GetRGBValues();
      centroid_matrix_ = pcd_container_->GetCentroidMatrix();
      scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
    } else {
      switch(pose_optimization_id_) {
        case 2:
          xyz_buffer_.clear();
          xyz_buffer_ = pcd_container_->GetXYZValuesOptWithMSM(glm::inverse(curr_pose_*centroid_matrix_));
          rgb_buffer_.clear();
          rgb_buffer_ = pcd_container_->GetRGBOptWithMSMValues();
          scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
          break;
        case 1:
          xyz_buffer_.clear();
          xyz_buffer_ = pcd_container_->GetXYZValuesOptWithSM(glm::inverse(curr_pose_*centroid_matrix_));
          rgb_buffer_.clear();
          rgb_buffer_ = pcd_container_->GetRGBOptWithSMValues();
          //curr_pose_ = pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_);
          //curr_pose_T_centroid = curr_pose_ * curr_pose_T_centroid;
          scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
          break;
        case 0:
          xyz_buffer_.clear();
          xyz_buffer_ = pcd_container_->GetXYZValues(glm::inverse(curr_pose_*centroid_matrix_));
          rgb_buffer_.clear();
          rgb_buffer_ = pcd_container_->GetRGBValues();
          //curr_pose_ = pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_);
          //curr_pose_T_centroid = curr_pose_ * curr_pose_T_centroid;
          //LOGE("Render : size %i", xyz.size());
          scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
          break;
        default:
          scene_->Render(pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(curr_pose_), xyz_buffer_, rgb_buffer_);
      }
    }
  }

  void SynchronizationApplication::OptimizeAndSaveToFolder(std::string folder_name) {
    StopPCDWorker();

    optimize_ = true;
    folder_name_ = folder_name;
    CreateSubFolders(folder_name);

    FrameToFrameScanMatcher ftfsm;
    MultiframeScanMatcher mfsm;
    LOGE("optimizationMethods_ case %i", optimizationMethods_);
    switch (optimizationMethods_) {
      case 0:
        LOGE("CASE 0");
        SavePCD(folder_name, "PCD/RAW/");
        ftfsm.Init(pcd_container_);
        ftfsm.Optimize();
        SavePCD(folder_name, "PCD/FTFSM/");
        break;
      case 1:
        LOGE("CASE 1");
        SavePCD(folder_name, "PCD/RAW/");
        mfsm.Init(pcd_container_);
        mfsm.Optimize();
        SavePCD(folder_name, "PCD/MFSM/");
        break;
      case 2:
        LOGE("CASE 2");
        SavePCD(folder_name, "PCD/RAW/");
        ftfsm.Init(pcd_container_);
        ftfsm.Optimize();
        SavePCD(folder_name, "PCD/FTFSM/");
        mfsm.Init(pcd_container_);
        mfsm.Optimize();
        SavePCD(folder_name, "PCD/MFSM/");
        break;
      default:
        break;
    }


    /*SavePCD(folder_name, "PCD/RAW/");
    UpdatePCDs();
    SavePCD(folder_name, "PCD/RAW2/");*/





    //std::clock_t start = std::clock();
    pcd_container_->OptimizeMesh();

    pcl::io::savePCDFile (folder_name + "Mesh/FTFSM.pcd", *pcd_container_->GetFTFSMMeshPCDFile());
    pcl::io::savePCDFile (folder_name + "Mesh/MFSM.pcd", *pcd_container_->GetMFSMMeshPCDFile());

    //int diff = (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000);
    show_msm_mesh_ = true;

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
                         reinterpret_cast<jint>((int)pcd_container_->pcd_container_.size()));

    //LOGE("Build sm and msm mesh stops after %i ms", diff);
  }

  void SynchronizationApplication::CreateSubFolders(std::string folder_name) {
    std::string dir_name = folder_name + "PCD/FTFSM";
    boost::filesystem::path dir = dir_name.c_str();
    boost::filesystem::create_directories(dir);
    dir_name = folder_name + "PCD/MFSM";
    dir = dir_name.c_str();
    boost::filesystem::create_directories(dir);
    dir_name = folder_name + "PCD/RAW";
    dir = dir_name.c_str();
    boost::filesystem::create_directories(dir);
    dir_name = folder_name + "PCD/RAW2";
    dir = dir_name.c_str();
    boost::filesystem::create_directories(dir);
    dir_name = folder_name + "Mesh";
    dir = dir_name.c_str();
    boost::filesystem::create_directories(dir);
  }

  void SynchronizationApplication::UpdatePCDs() {

    int lastIndex = pcd_container_->GetPCDContainerLastIndex();
    for (int i = 0; i <= lastIndex; i++) {
      pcd_container_->pcd_container_[i]->Update();
    }

  }

  void SynchronizationApplication::StopPCDWorker() {
    pcd_worker_->Stop();
    LOGE("stop PCD worker");
    while(pcd_worker_->IsRunning()) {
      // wait until all processes finished
    }
  }

  void SynchronizationApplication::StartPCDWorker() {
    optimize_ = false;
    pcd_worker_->Stop();
    LOGE("stop PCD worker");
    while(pcd_worker_->IsRunning()) {
      // wait until all processes finished
    }
    pcd_container_->ResetPCD();
    pcd_worker_->Start();
  }

  void SynchronizationApplication::SavePCD(std::string folder_name, std::string subfolder_name) {

    int lastIndex = pcd_container_->GetPCDContainerLastIndex();

    for (int i = 0; i <= lastIndex; i++) {

      if (subfolder_name == "PCD/RAW/") {
        std::string dir_path = folder_name + subfolder_name;
        char filename[1024];
        sprintf(filename, "%s/%05d.pcd", dir_path.c_str(), i);
        pcd_container_->pcd_container_[i]->SaveAsPCD(filename);
      }

      if (subfolder_name == "PCD/RAW2/") {
        std::string dir_path = folder_name + subfolder_name;
        char filename[1024];
        sprintf(filename, "%s/%05d.pcd", dir_path.c_str(), i);
        pcd_container_->pcd_container_[i]->SaveAsPCD(filename);
      }

      if (subfolder_name == "PCD/FTFSM/") {
        std::string dir_path = folder_name + subfolder_name;
        char filename[1024];
        sprintf(filename, "%s/%05d.pcd", dir_path.c_str(), i);
        pcd_container_->pcd_container_[i]->SaveAsPCDWithFTFSMPose(filename);
      }
      if (subfolder_name == "PCD/MFSM/") {
        std::string dir_path = folder_name + subfolder_name;
        char filename[1024];
        sprintf(filename, "%s/%05d.pcd", dir_path.c_str(), i);
        pcd_container_->pcd_container_[i]->SaveAsPCDWithMFSMPose(filename);
      }


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

  void SynchronizationApplication::SetRangeValue(float range) {
    range_ = range;
    pcd_worker_->SetRangeValue(range);
    LOGE("RANGE VALUE: %f", range_);
  }

  void SynchronizationApplication::ShowUnOPTMesh() {
    pose_optimization_id_ = 0;
    LOGE("Show Unoptimized Mesh");
  }

  void SynchronizationApplication::ShowSMMesh() {
    pose_optimization_id_ = 1;
    LOGE("Show SM Mesh");
  }

  void SynchronizationApplication::ShowMSMMesh() {
    pose_optimization_id_ = 2;
    LOGE("Show MSM Mesh");
  }

  void SynchronizationApplication::SetBackgroundColorBlack(bool on) {
    scene_->SetBackgroundColor(on);
  }

  void SynchronizationApplication::SetGridOn(bool on) {
    scene_->SetGridOn(on);
  }

  void SynchronizationApplication::SetOptimizationMethods(int opt) {
    optimizationMethods_ = opt;
  }

}  // namespace rgb_depth_sync
