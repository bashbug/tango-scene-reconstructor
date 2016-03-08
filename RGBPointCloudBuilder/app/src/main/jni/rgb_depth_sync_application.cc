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

    if (!(*optimize_poses_process_started_)) {
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
    if (!(*optimize_poses_process_started_)) {
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
          pcd_count_++;
        } else {
          LOGE("ss_T_device_xyz_timestamp pose not valid");
        }
      }
    }
  }

  // This function routes onPoseAvailable callbacks to the application object for
// handling.
//
// @param context, context will be a pointer to a PointCloudApp
//        instance on which to call callbacks.
// @param pose, pose data to route to onPoseAvailable function.
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

  int SynchronizationApplication::TangoInitialize(JNIEnv* env, jobject caller_activity) {
    // The first thing we need to do for any Tango enabled application is to
    // initialize the service. We'll do that here, passing on the JNI environment
    // and jobject corresponding to the Android activity that is calling us.
    return TangoService_initialize(env, caller_activity);
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

    // Note that it's super important for AR applications that we enable low
    // latency imu integration so that we have pose information available as
    // quickly as possible.
    /*ret = TangoConfig_setBool(tango_config_, "config_enable_low_latency_imu_integration", true);
    if (ret != TANGO_SUCCESS) {
      LOGE("Failed to enable low latency imu integration.");
      return ret;
    }*/

    optimize_poses_process_started_ = std::make_shared<std::atomic<bool>>(false);
    pcd_mtx_ = std::make_shared<std::mutex>();
    consume_pcd_ = std::make_shared<std::condition_variable>();
    xyz_mtx_ = std::make_shared<std::mutex>();
    consume_xyz_ = std::make_shared<std::condition_variable>();

    int32_t max_point_cloud_elements;
    ret = TangoConfig_getInt32(tango_config_, "max_point_cloud_elements",
                                   &max_point_cloud_elements);

    if(ret != TANGO_SUCCESS) {
      LOGE("Failed to query maximum number of point cloud elements.");
      return ret;
    }

    TangoSupport_createPointCloudManager(max_point_cloud_elements, &xyz_manager_);
    TangoSupport_createImageBufferManager(TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP, 1280, 720, &yuv_manager_);

    curr_index_ = -1;
    prev_index_ = -1;
    first_index_ = true;

    save_pcd_ = false;
    start_pcd_ = false;
    pcd_count_ = 0;
    img_count_ = 0;
    pcd_container_optimized_ = false;

    TangoCameraIntrinsics depth_camera_intrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_DEPTH, &depth_camera_intrinsics);
    /*LOGE("depth camera width: %i", depth_camera_intrinsics.width);
    LOGE("depth camera height: %i", depth_camera_intrinsics.height);
    LOGE("depth camera fx: %f", depth_camera_intrinsics.fx);
    LOGE("depth camera fy: %f", depth_camera_intrinsics.fy);
    LOGE("depth camera cx: %f", depth_camera_intrinsics.cx);
    LOGE("depth camera cy: %f", depth_camera_intrinsics.cy);*/

    TangoCameraIntrinsics color_camera_intrinsics;
    TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &color_camera_intrinsics);
    /*LOGE("depth camera width: %i", color_camera_intrinsics.width);
    LOGE("depth camera height: %i", color_camera_intrinsics.height);
    LOGE("depth camera fx: %f", color_camera_intrinsics.fx);
    LOGE("depth camera fy: %f", color_camera_intrinsics.fy);
    LOGE("depth camera cx: %f", color_camera_intrinsics.cx);
    LOGE("depth camera cy: %f", color_camera_intrinsics.cy);*/

    pcd_container_ = new rgb_depth_sync::PCDContainer(pcd_mtx_, consume_pcd_);
    pcd_worker_ = new rgb_depth_sync::PCDWorker(xyz_mtx_, consume_xyz_, pcd_container_, xyz_manager_, yuv_manager_);

    std::thread pcd_worker_thread(&rgb_depth_sync::PCDWorker::OnPCDAvailable, pcd_worker_);
    pcd_worker_thread.detach();

    slam_ = new rgb_depth_sync::Slam3D(pcd_container_, pcd_mtx_, consume_pcd_, optimize_poses_process_started_);

    pose_data_ = PoseData::GetInstance();
    pose_data_->SetColorCameraIntrinsics(color_camera_intrinsics);

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

    // TangoService_getPoseAtTime function is used for query device extrinsics
    // as well. We use timestamp 0.0 and the target frame pair to get the
    // extrinsics from the sensors.
    //
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

    if (!(*optimize_poses_process_started_)) {

      curr_index_ = pcd_container_->GetPCDContainerLastIndex();

      glm::mat4 curr_pose = pose_data_->GetLatestPoseMatrix();
      std::vector<float> xyz = pcd_container_->GetXYZValues(glm::inverse(curr_pose));
      std::vector<uint8_t> rgb = pcd_container_->GetRGBValues();

      bool newData = true;
      /*if (curr_index_ == prev_index_) {
        newData = false;
        //LOGE("NO new data");
        if (xyz.size() > 0 && rgb.size() > 0) {
          /*pose_ = pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(pose_data_->GetLatestPoseMatrix());
          main_scene_.Render(pose_, pose_, xyz, rgb, newData);
          if (first_index_) {
            first_index_ = false;
          }
        }
      } else {*/
        //LOGE("YES new data");
        if (xyz.size() > 0 && rgb.size() > 0) {
          pose_ = pose_data_->GetExtrinsicsAppliedOpenGLWorldFrame(pose_data_->GetLatestPoseMatrix());
          //LOGE("size: %i", xyz.size());
          scene_->Render(pose_, pose_, xyz, rgb);
          if (first_index_) {
            first_index_ = false;
          }
        }
      //}

      if (!first_index_) {
        prev_index_ = curr_index_;
      }

      /*icp_ = glm::mat4();
      /*std::vector<float> xyz;
      std::vector<uint8_t> rgb;
      glm::mat4 ss_T_device;
      pcd_container_->GetXYZRGBValues(&xyz, &rgb, &ss_T_device);

      glm::mat4 opengl_T_device = conversion_->OpenGL_T_Device(ss_T_device);
      glm::mat4 opengl_T_device_T_opengl_camera = conversion_->OpenGL_T_device_T_OpenGLCamera(ss_T_device);

      if (xyz.size() > 0 && rgb.size() > 0) {
        scene_->Render(opengl_T_device,
                       opengl_T_device_T_opengl_camera,
                       icp_,
                       xyz,
                       rgb);
      }
      int index = pcd_container_->GetPCDContainerLastIndex();
      if (index >= 0) {
        pcd_ = (*(pcd_container_->GetPCDContainer()))[index];
        if (pcd_ != nullptr) {
          scene_->Render(pcd_->GetOpenGL_T_OpenGLCameraPose(),
                         pcd_->GetOpenGL_T_RGB(), icp_,
                         pcd_->GetXYZValues(),
                         pcd_->GetRGBValues());
        }
      }*/
    }
  }

  void SynchronizationApplication::OptimizePoseGraph(bool on) {
    if (on) {
      optimize_poses_process_started_ = std::make_shared<std::atomic<bool>>(true);
      std::thread slam_thread(&rgb_depth_sync::Slam3D::OptimizeGraph, slam_);
      slam_thread.detach();
      pcd_container_optimized_ = true;
    }
  }

  void SynchronizationApplication::StartPCD(bool on) {
    if(on && !start_pcd_) {
      start_pcd_ = true;
      // reset pcd_container data
      pcd_container_->ResetPCD();
      // start slam thread with loop closure detection
      slam_->StartOnFrameAvailableThread();
      std::thread slam_thread(&rgb_depth_sync::Slam3D::OnPCDAvailable, slam_);
      slam_thread.detach();
      LOGE("START PCD pcd_counter: %i, img_counter: %i", pcd_count_, img_count_);
    }
  }

  void SynchronizationApplication::StopPCD(bool on) {
    if(on && start_pcd_) {
      // start slam thread with loop closure detection
      slam_->StopOnFrameAvailableThread();
      start_pcd_ = false;
      pcd_worker_->StopPCDWorker();
      LOGE("STOP PCD pcd_counter: %i, img_counter: %i", pcd_count_, img_count_);
    }
  }

  void SynchronizationApplication::SavePCD(bool on) {
    if(on) {
      /*boost::system::error_code* error;
      boost::filesystem::path path = "/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/";
      boost::filesystem::detail::status(path, error);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB> trans;
      int lastIndex = pcd_container_->GetPCDContainerLastIndex();*/

      /*for (int i = 0; i < lastIndex; i++) {
        pcl::transformPointCloud(*(*(pcd_container_->GetPCDContainer()))[i]->GetPCD(), trans, (*(pcd_container_->GetPCDContainer()))[i]->GetTransformationMatrix());
        *out += trans;
      }*/

      //pcl::io::savePCDFileBinary("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000000_.PCD", *(pcd_container_->GetMergedPCD()));

      /*pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud (out);
      sor.setLeafSize (0.001f, 0.001f, 0.001f);
      sor.filter (*out_filtered);

      pcl::io::savePCDFileBinary("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000000_f.PCD", *out_filtered);*/

      //pcl::io::savePCDFile("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000000.pcd", *(*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPCD());
      /*pcl::PCDWriter writer;
      writer.write<pcl::PointXYZRGB> ("/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/00000000_.pcd", *(*(pcd_container_->GetPCDContainer()))[lastIndex]->GetPCD());*/

      PCDFileWriter pcd_file_writer;
      IMGFileWriter img_file_writer;

      int lastIndex = pcd_container_->GetPCDContainerLastIndex();

      if (pcd_container_optimized_) {
        for (int i = 0; i <= lastIndex; i++) {
          pcd_file_writer.SetPCDRGBData(
              pcd_container_->pcd_container_[i]->GetPCD(),
              pcd_container_->pcd_container_[i]->GetTranslation(),
              pcd_container_->pcd_container_[i]->GetRotation());
          pcd_file_writer.SetUnordered();
          // save files asynchron
          pcd_file_writer.SaveToFile("PCD_opt", i);
          // save img asynchron
          //img_file_writer.SaveToFile(i, (*(pcd_container_->GetPCDContainer()))[i]->GetFrame());
        }
      } else {
        for (int i = 0; i <= lastIndex; i++) {
          pcd_file_writer.SetPCDRGBData(
              pcd_container_->pcd_container_[i]->GetPCD(),
              pcd_container_->pcd_container_[i]->GetTranslation(),
              pcd_container_->pcd_container_[i]->GetRotation());
          pcd_file_writer.SetUnordered();
          // save files asynchron
          pcd_file_writer.SaveToFile("PCD", i);
          // save img asynchron
          //img_file_writer.SaveToFile(i, (*(pcd_container_->GetPCDContainer()))[i]->GetFrame());
        }
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

  void SynchronizationApplication::SetSocket(std::string addr, int port) {
    socket_addr_ = addr;
    socket_port_ = port;
  }
}  // namespace rgb_depth_sync
