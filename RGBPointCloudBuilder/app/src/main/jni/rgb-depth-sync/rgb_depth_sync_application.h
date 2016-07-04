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

#ifndef RGB_DEPTH_SYNC_APPLICATION_H_
#define RGB_DEPTH_SYNC_APPLICATION_H_

#include <tango_support_api.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <string>
#include <memory>
#include <boost/filesystem.hpp>
#include <tango_client_api.h>
#include <tango-gl/util.h>
#include <tango-gl/axis.h>
#include <tango-gl/camera.h>
#include <tango-gl/color.h>
#include <tango-gl/gesture_camera.h>
#include <tango-gl/grid.h>
#include <tango-gl/frustum.h>
#include <tango-gl/trace.h>
#include <tango-gl/transform.h>
#include "jni.h"
#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/img_file_writer.h"
#include "rgb-depth-sync/pcd.h"
#include "rgb-depth-sync/pose_data.h"
#include "rgb-depth-sync/pcd_container.h"
#include "rgb-depth-sync/pcd_worker.h"
#include "rgb-depth-sync/pcd_file_reader.h"
#include "rgb-depth-sync/pcd_file_writer.h"
#include "rgb-depth-sync/frame_to_frame_scan_matcher.h"
#include "rgb-depth-sync/multiframe_scan_matcher.h"
#include "rgb-depth-sync/scene.h"
#include "rgb-depth-sync/conversion.h"

namespace rgb_depth_sync {

  class SynchronizationApplication {
    public:
      SynchronizationApplication();
      ~SynchronizationApplication();
      void OnTangoServiceConnected(JNIEnv* env, jobject binder);
      int TangoInitialize(JNIEnv* env, jobject caller_activity, JavaVM* javaVM);
      // Setup the configuration file for the Tango Service. .
      int TangoSetupConfig();
      // Sets the callbacks for OnXYZijAvailable
      int TangoConnectCallbacks();
      // Connect to Tango Service.
      // This function will start the Tango Service pipeline, in this case, it will
      // start Depth Sensing callbacks.
      int TangoConnect();
      // Queries and sets the camera transforms between different sensors of
      // Project Tango Device that are required to project Point cloud onto
      // Image plane.
      int TangoSetIntrinsicsAndExtrinsics();
      // Disconnect from Tango Service.
      void TangoDisconnect();
      // Set render camera's viewing angle, first person, third person or top down.
      //
      // @param: camera_type, camera type includes first person, third person and
      //         top down
      void SetBackgroundColorBlack(bool on);
      void SetGridOn(bool on);
      void SetOptimizationMethods(int opt);
      void SetCameraType(tango_gl::GestureCamera::CameraType camera_type);
      void StartPCDWorker();
      void StopPCDWorker();
      void SavePCD(bool on);
      void ShowSMMesh();
      void ShowMSMMesh();
      void ShowUnOPTMesh();
      void SetRangeValue(float range);
      void OptimizeAndSaveToFolder(std::string folder_name);
      void CreateSubFolders(std::string folder_name);
      // Touch event passed from android activity. This function only supports two
      // touches.
      //
      // @param: touch_count, total count for touches.
      // @param: event, touch event of current touch.
      // @param: x0, normalized touch location for touch 0 on x axis.
      // @param: y0, normalized touch location for touch 0 on y axis.
      // @param: x1, normalized touch location for touch 1 on x axis.
      // @param: y1, normalized touch location for touch 1 on y axis.
      void OnTouchEvent(int touch_count, tango_gl::GestureCamera::TouchEvent event, float x0, float y0, float x1, float y1);
      // Inititalizes all the OpenGL resources required to render a Depth Image on
      // Top of an RGB image.
      void InitializeGLContent();
      // Setup the view port width and height.
      void SetViewPort(int width, int height);
      // Main Render loop.
      void Render();
      // Release all OpenGL resources that are allocated in this app.
      void FreeGLContent();
      // Set whether to use GPU or CPU upsampling
      void SetSocket(std::string addr, int port);
      // Callback for image data that come in from the Tango service.
      // @param buffer The image data returned by the service.
      void OnFrameAvailable(const TangoImageBuffer* buffer);
      // Callback for point clouds that come in from the Tango service.
      // @param xyz_ij The point cloud returned by the service.
      void OnXYZijAvailable(const TangoXYZij* xyz_ij);

      void OnPoseAvailable(const TangoPoseData* pose);

      void UpdatePCDs();

    private:
      void SavePCD(std::string folder_name, std::string subfolder_name);
      int screen_width_, screen_height_;
      int pcd_count_;
      int img_count_;
      TangoConfig tango_config_;
      bool pcd_container_optimized_;
      bool pcd_container_optimized_mf_;
      bool save_pcd_;
      bool start_pcd_;
      //std::shared_ptr<std::atomic<bool>> optimize_poses_process_started_;
      std::shared_ptr<std::mutex> pcd_mtx_;
      std::shared_ptr<std::condition_variable> consume_pcd_;
      std::string socket_addr_;
      int socket_port_;
      PCDWorker* pcd_worker_;
      PCD* pcd_;
      glm::mat4 icp_;
      PCDContainer* pcd_container_;
      Scene* scene_;
      TangoSupportPointCloudManager* xyz_manager_;
      TangoSupportImageBufferManager* yuv_manager_;
      Conversion* conversion_;
      TangoXYZij* xyz_;
      TangoImageBuffer* yuv_;
      std::mutex pose_mutex_;
      std::mutex optimize_mutex_;
      PoseData* pose_data_;
      bool new_xyz_data;
      bool new_yuv_data;
      std::vector<float> xyz_buffer_;
      std::vector<uint8_t> rgb_buffer_;
      glm::mat4 curr_pose_;
      glm::mat4 pose_;
      int curr_index_;
      int prev_index_;
      bool first_index_;
      int point_cloud_count_;
      std::shared_ptr<std::mutex> xyz_mtx_;
      std::shared_ptr<std::condition_variable> consume_xyz_;
      bool show_sm_mesh_, show_msm_mesh_, show_unopt_mesh_;
      float range_;
      std::string folder_name_;
      bool optimize_;
      JNIEnv* env_;
      jobject caller_activity_;
      JavaVM* javaVM_;
      jclass activity_class_;
      int pose_optimization_id_;
      glm::mat4 centroid_matrix_;
      bool backgroundColorBlack_;
      int optimizationMethods_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_APPLICATION_H_
