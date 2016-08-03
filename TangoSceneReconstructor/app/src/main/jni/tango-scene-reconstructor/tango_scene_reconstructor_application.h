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

#ifndef TANGOSCENERECONSTRUCTOR_TANGO_SCENE_RECONSTRUCTOR_APPLICATION_H_
#define TANGOSCENERECONSTRUCTOR_TANGO_SCENE_RECONSTRUCTOR_APPLICATION_H_

#include <mutex>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <string>
#include <memory>

#include "jni.h"

#include <tango_support_api.h>
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

#include "tango-scene-reconstructor/util.h"
#include "tango-scene-reconstructor/pose_data.h"
#include "tango-scene-reconstructor/point_cloud_manager.h"
#include "tango-scene-reconstructor/io/PCD.h"
#include "tango-scene-reconstructor/io/VTK.h"
#include "tango-scene-reconstructor/scan_matcher/frame_to_frame_scan_matcher.h"
#include "tango-scene-reconstructor/scan_matcher/multiframe_scan_matcher.h"
#include "tango-scene-reconstructor/scene.h"
#include "tango-scene-reconstructor/conversion.h"

namespace tango_scene_reconstructor {

  class TangoSceneReconstructorApplication {
    public:
      TangoSceneReconstructorApplication();
      ~TangoSceneReconstructorApplication();
      void OnTangoServiceConnected(JNIEnv* env, jobject binder);
      int TangoInitialize(JNIEnv* env, jobject caller_activity, JavaVM* javaVM);
      int TangoSetupConfig();
      int TangoConnectCallbacks();
      int TangoConnect();
      int TangoSetIntrinsicsAndExtrinsics();
      void TangoDisconnect();
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
      void Optimize();
      void SaveToFolder(std::string folder_name, int save_mode);
      void OnTouchEvent(int touch_count, tango_gl::GestureCamera::TouchEvent event, float x0, float y0, float x1, float y1);
      void InitializeGLContent();
      void SetViewPort(int width, int height);
      void Render();
      void FreeGLContent();
      void SetSocket(std::string addr, int port);
      void OnFrameAvailable(const TangoImageBuffer* buffer);
      void OnXYZijAvailable(const TangoXYZij* xyz_ij);
      void OnPoseAvailable(const TangoPoseData* pose);
      void UpdatePCDs();

    private:

      JNIEnv* env_;
      jobject caller_activity_;
      JavaVM* javaVM_;
      jclass activity_class_;

      PointCloudManager* point_cloud_manager_;
      PoseData* pose_data_;
      Scene* scene_;
      TangoSupportPointCloudManager* xyz_manager_;
      TangoSupportImageBufferManager* yuv_manager_;

      TangoConfig tango_config_;

      std::mutex pose_mutex_;
      std::shared_ptr<std::mutex> xyz_mtx_;
      std::shared_ptr<std::condition_variable> consume_xyz_;

      std::vector<float> xyz_buffer_;
      std::vector<uint8_t> rgb_buffer_;

      glm::mat4 curr_pose_;

      bool optimize_;
      glm::mat4 centroid_matrix_;
      int pose_optimization_id_;
      int optimizationMethods_;
  };

} // namespace tango_scene_reconstructor

#endif // TANGOSCENERECONSTRUCTOR_TANGO_SCENE_RECONSTRUCTOR_APPLICATION_H_
