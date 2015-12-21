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

#include <jni.h>

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

#include <projectiveScanMatcher3d/projectiveScanMatcher3d.h>
#include <projectiveImage/sphericalProjectiveImage.h>

#include <Eigen/Geometry>

#include "rgb-depth-sync/util.h"
#include "rgb-depth-sync/color_image.h"
#include "rgb-depth-sync/range_image.h"
#include "rgb-depth-sync/point_cloud_data.h"
#include "rgb-depth-sync/point_cloud_container.h"
#include "rgb-depth-sync/pose_container.h"
#include "rgb-depth-sync/scene.h"

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

  // This thread safe class is the main application for Synchronization.
  // It can be instantiated in the JNI layer and use to pass information back and
  // forth between Java. The class also manages the application's lifecycle and
  // interaction with the Tango service. Primarily, this involves registering for
  // callbacks and passing on the necessary information to stored objects. It also
  // takes care of passing a vector container which has a pointer to the
  // latest point cloud buffer that is to used for rendering.
  //  To reduce the number of point cloud data copies between callback and render
  // threads we use three buffers which are synchronously exchanged between each
  // other so that the render loop always contains the latest point cloud data.
  // 1. Callback buffer : The buffer to which pointcloud data received from Tango
  // Service callback is copied out.
  // 2. Shared buffer: This buffer is used to share the data between Service
  // callback and Render loop
  // 3. Render Buffer: This buffer is used in the renderloop to project point
  // cloud data to a 2D image plane which is of the same size as RGB image. We
  // also make sure that this buffer contains the latest point cloud data that is
  //  received from the call back.

  class SynchronizationApplication {
    public:
      SynchronizationApplication();
      ~SynchronizationApplication();
      // Initialize the Tango Service, this function starts the communication
      // between the application and the Tango Service.
      // The activity object is used for checking if the API version is outdated
      int TangoInitialize(JNIEnv* env, jobject caller_activity);
      int TangoSetPCDSave(bool isChecked);
      int TangoSetPCDSend(bool isChecked);
      int TangoStoreImage(bool store);
      // Setup the configuration file for the Tango Service. .
      int TangoSetupConfig();
      // Associate the texture generated from an Opengl context to which the color
      // image will be updated to.
      int TangoConnectTexture();
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

      // Touch event passed from android activity. This function only supports two
      // touches.
      //
      // @param: touch_count, total count for touches.
      // @param: event, touch event of current touch.
      // @param: x0, normalized touch location for touch 0 on x axis.
      // @param: y0, normalized touch location for touch 0 on y axis.
      // @param: x1, normalized touch location for touch 1 on x axis.
      // @param: y1, normalized touch location for touch 1 on y axis.
      void OnTouchEvent(int touch_count, tango_gl::GestureCamera::TouchEvent event,
                        float x0, float y0, float x1, float y1);
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
      void SetDepthMap(bool on);
      void SetRGBMap(bool on);
      void SetSocket(std::string addr, int port);
      void SetStartPCDRecording(bool on);
      void SetSendPCDRecording(bool on);
      // Callback for image data that come in from the Tango service.
      // @param buffer The image data returned by the service.
      void OnFrameAvailable(const TangoImageBuffer* buffer);
      // Callback for point clouds that come in from the Tango service.
      // @param xyz_ij The point cloud returned by the service.
      void OnXYZijAvailable(const TangoXYZij* xyz_ij);
      void OnPoseAvailable(const TangoPoseData* pose);

    private:
      glm::mat4 convertEigenToGLMPose(Eigen::Isometry3f eigen_pose);
      Eigen::Isometry3f convertGLMToEigenPose(glm::mat4 glm_pose);
      glm::mat4 GetExtrinsicsAppliedOpenGLWorldFrame(const glm::mat4 pose_matrix);
      void SetRGBBuffer();
      void StoreImage();
      void Yuv2Rgb(uint8_t yValue, uint8_t uValue, uint8_t vValue,
                   uint8_t* r, uint8_t* g, uint8_t* b, uint32_t* rgb);
      void WriteByteToPPM(const char* filename, std::vector<uint8_t> rgb_bytebuffer,
                          size_t w, size_t h);

      PointCloudContainer* point_cloud_container_;
      PoseContainer* pose_container_;
      ColorImage* color_image_;
      RangeImage* range_image_;
      TangoConfig tango_config_;
      glm::mat4 device_T_color_;
      glm::mat4 device_T_depth_;
      glm::mat4 color_T_device_;
      glm::mat4 OW_T_SS_;
      float screen_width_;
      float screen_height_;
      // This is the buffer to which point cloud data from TangoService callback
      // gets copied out to.
      // The data is an array of packed coordinate triplets, x,y,z as floating point
      // values. With the unit in landscape orientation, screen facing the user:
      // +Z points in the direction of the camera's optical axis, and is measured
      // perpendicular to the plane of the camera.
      // +X points toward the user's right, and +Y points toward the bottom of
      // the screen.
      // The origin is the focal centre of the color camera.
      // The output is in units of metres.
      std::vector<float> callback_point_cloud_buffer_;
      // The buffer of point cloud data which is shared between TangoService
      // callback and render loop.
      std::vector<float> shared_point_cloud_buffer_;
      // This buffer is used in the render loop to project point cloud data to
      // a 2D image plane which is of the same size as RGB image.
      std::vector<float> render_point_cloud_buffer_;
      // Time of capture of the current depth data (in seconds).
      double depth_timestamp_;
      double color_timestamp_;
      // Mutex for protecting the point cloud data. The point cloud data is shared
      // between update call which is called from render loop and
      // TangoService callback thread.
      std::mutex point_cloud_mutex_;
      std::mutex rgb_mutex_;
      // This signal is used to notify update call if there is a new
      // point cloud buffer available and swap the shared and render buffers
      // accordingly.
      bool swap_point_cloud_buffer_signal_;
      bool render_depth_map_;
      bool render_range_image_;
      bool render_image_;
      bool update_point_cloud_container;
      bool send_point_cloud_container;
      std::vector<uint8_t> callback_yuv_buffer_;
      std::vector<uint8_t> shared_yuv_buffer_;
      std::vector<uint8_t> render_yuv_buffer_;
      std::vector<uint8_t> rgb_map_buffer_;
      std::vector<uint32_t> rgb_pcd_buffer_;
      bool swap_rgb_buffer_signal_;
      bool swap_yuv_buffer_signal_;
      std::mutex yuv_buffer_mutex_;
      std::mutex rgb_buffer_mutex_;
      std::mutex pose_mutex_;
      int count = 0;
      PointCloudData* pcd_;
      int current_timestamp_;
      int previous_timestamp_;
      size_t yuv_width_;
      size_t yuv_height_;
      size_t yuv_size_;
      size_t uv_buffer_offset_;
      bool store_point_clouds_;
      bool store_image_;
      bool send_point_clouds_;
      std::string socket_addr_;
      int socket_port_;

      // Point Cloud
      Scene* scene_;

      Eigen::Isometry3f icpPose_;
      ProjectiveScanMatcher3d* projective_scan_matcher_;
      SphericalProjectiveImage* projective_image_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_APPLICATION_H_
