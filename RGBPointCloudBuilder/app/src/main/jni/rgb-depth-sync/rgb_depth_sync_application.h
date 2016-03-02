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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <boost/make_shared.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <mutex>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <memory>
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
#include "rgb-depth-sync/pcd_container.h"
#include "rgb-depth-sync/pcd_worker.h"
#include "rgb-depth-sync/pcd_file_reader.h"
#include "rgb-depth-sync/pcd_file_writer.h"
#include "rgb-depth-sync/scene.h"
#include "rgb-depth-sync/slam3d.h"
#include "rgb-depth-sync/conversion.h"

namespace {
// We want to represent the device properly with respect to the ground so we'll
// add an offset in z to our origin. We'll set this offset to 1.3 meters based
// on the average height of a human standing with a Tango device. This allows us
// to place a grid roughly on the ground for most users.
  const glm::vec3 kHeightOffset = glm::vec3(0.0f, 1.3f, 0.0f);

// Color of the motion tracking trajectory.
  const tango_gl::Color kTraceColor(0.22f, 0.28f, 0.67f);

// Color of the ground grid.
  const tango_gl::Color kGridColor(0.85f, 0.85f, 0.85f);

// Frustum scale.
  const glm::vec3 kFrustumScale = glm::vec3(0.4f, 0.3f, 0.5f);
}  // namespace

namespace rgb_depth_sync {

  class SynchronizationApplication {
    public:
      SynchronizationApplication();
      ~SynchronizationApplication();
      int TangoInitialize(JNIEnv* env, jobject caller_activity);
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
      void SetCameraType(tango_gl::GestureCamera::CameraType camera_type);
      void StartPCD(bool on);
      void StopPCD(bool on);
      void SavePCD(bool on);
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
      void OptimizePoseGraph(bool on);
      // Set whether to use GPU or CPU upsampling
      void SetSocket(std::string addr, int port);
      // Callback for image data that come in from the Tango service.
      // @param buffer The image data returned by the service.
      void OnFrameAvailable(const TangoImageBuffer* buffer);
      // Callback for point clouds that come in from the Tango service.
      // @param xyz_ij The point cloud returned by the service.
      void OnXYZijAvailable(const TangoXYZij* xyz_ij);

    private:
      int screen_width_, screen_height_;
      int pcd_count_;
      int img_count_;
      TangoConfig tango_config_;
      bool pcd_container_optimized_;
      bool save_pcd_;
      bool start_pcd_;
      std::shared_ptr<std::atomic<bool>> optimize_poses_process_started_;
      std::shared_ptr<std::mutex> pcd_mtx_;
      std::shared_ptr<std::condition_variable> consume_pcd_;
      std::string socket_addr_;
      int socket_port_;
      PCDWorker* pcd_worker_;
      PCD* pcd_;
      glm::mat4 icp_;
      PCDContainer* pcd_container_;
      Scene* scene_;
      Slam3D* slam_;
      TangoSupportPointCloudManager* xyz_manager_;
      TangoSupportImageBufferManager* yuv_manager_;
      Conversion* conversion_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_APPLICATION_H_
