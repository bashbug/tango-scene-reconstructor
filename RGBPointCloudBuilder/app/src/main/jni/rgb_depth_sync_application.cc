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


#include <tango-gl/conversions.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <rgb-depth-sync/rgb_depth_sync_application.h>

namespace rgb_depth_sync {

// We could do this conversion in a fragment shader if all we care about is
// rendering, but we show it here as an example of how people can use RGB data
// on the CPU.
void SynchronizationApplication::Yuv2Rgb(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t* r,
                    uint8_t* g, uint8_t* b, uint32_t* rgb) {
  *r = yValue + (1.370705f * (vValue - 128.0f));
  *g = yValue - (0.698001f * (vValue - 128.0f)) - (0.337633f * (uValue - 128.0f));
  *b = yValue + (1.732446f * (uValue - 128.0f));

  if(*r < 0) {
    *r = 0;
  }
  if(*r > 255) {
    *r = 255;
  }

  if(*g < 0) {
    *g = 0;
  }
  if(*g > 255) {
    *g = 255;
  }

  if(*b < 0) {
    *b = 0;
  }
  if(*b > 255) {
    *b = 255;
  }

  *rgb = ((uint32_t) (*r)) << 16 | ((uint32_t) (*g)) << 8 | (uint32_t) (*b);
}

void SynchronizationApplication::WriteByteToPPM(const char* filename, std::vector<uint8_t> rgb_bytebuffer, size_t w, size_t h){

    FILE* file = fopen(filename, "wb");
    // ppm header
    fprintf(file, "P6\n%zu %zu\n255\n", w, h);
    // image data as unsigned char r,g,b values
    fwrite(&rgb_bytebuffer[0], rgb_bytebuffer.size(), sizeof(uint8_t), file);
    fclose(file);
};

void SynchronizationApplication::WriteByteArrayToBitmap(char* filename, std::vector<uint8_t> rgb_bytebuffer, size_t width, size_t height) {

    /*size_t padding = 4 - ((width * 3) % 4);
    if (padding == 4) {
      padding = 0;
    }

    size_t bmp_file_size = ((width * 3) + padding) * height + 54; // 54 = header size

    size_t bmp_file_header[3];
    bmp_file_header[0] = bmp_file_size;
    bmp_file_header[1] = 0; // reserved 2x2bytes
    bmp_file_header[2] = 54; // start of raw rgb image data offset

    size_t bmp_info_header[9];
    bmp_info_header[0] = 40; // size of this header in bytes
    bmp_info_header[1] = width; // witdh of image in pixel
    bmp_info_header[2] = height; // height of image in pixel
    bmp_info_header[3] = 0; // no compression
    bmp_info_header[4] = bmp_file_size - 54; // size of raw image data without header
    bmp_info_header[5] = 0; // the horizontal resolution of the image. (pixel per meter, signed integer)
    bmp_info_header[6] = 0; // the vertical resolution of the image. (pixel per meter, signed integer)
    bmp_info_header[7] = 0; // number of colors in the color palette. (0 default)
    bmp_info_header[8] = 0; // number of important colors used, (default 0, every color is important)

    FILE* file = fopen(filename, "wb");

    // Write header
    fprintf(file, "BM"); // header field used to identify the bmp

    // write file header
    for (int i = 0; i <= 4; i++) {
        fprintf(file, "%c", bmp_file_header[i] & 0x000000FF); // 1. byte
        fprintf(file, "%c", (bmp_file_header[i] & 0x0000FF00) >> 8); // 2. byte
        fprintf(file, "%c", (bmp_file_header[i] & 0x00FF0000) >> 16); // 3. byte
        fprintf(file, "%c", (bmp_file_header[i] & 0xFF000000) >> 24); // 4. byte
    }

    // write info header first part
    for (int i = 0; i <= 1; i++) {
        fprintf(file, "%c", bmp_info_header[i] & 0x000000FF); // 1. byte
        fprintf(file, "%c", (bmp_info_header[i] & 0x0000FF00) >> 8); // 2. byte
        fprintf(file, "%c", (bmp_info_header[i] & 0x00FF0000) >> 16); // 3. byte
        fprintf(file, "%c", (bmp_info_header[i] & 0xFF000000) >> 24); // 4. byte
    }

    fprintf(file, "%c", 1); // color plane (1 default) 1 byte
    fprintf(file, "%c", 0); // 1 byte
    fprintf(file, "%c", 24); // 1 byte
    fprintf(file, "%c", 0); // 1 byte

    // write info header
    for (int i = 0; i <= 1; i++) {
        fprintf(file, "%c", bmp_info_header[i] & 0x000000FF); // 1. byte
        fprintf(file, "%c", (bmp_info_header[i] & 0x0000FF00) >> 8); // 2. byte
        fprintf(file, "%c", (bmp_info_header[i] & 0x00FF0000) >> 16); // 3. byte
        fprintf(file, "%c", (bmp_info_header[i] & 0xFF000000) >> 24); // 4. byte
    }*/

    FILE* file = fopen(filename, "wb");

    // write raw image data
    for (int i = 0; i <= rgb_bytebuffer.size()-3; i = i+3) {
        unsigned char red = rgb_bytebuffer.at(i);
        unsigned char green = rgb_bytebuffer.at(i+1);
        unsigned char blue = rgb_bytebuffer.at(i+2);
        fprintf(file, "%c", red);
        fprintf(file, "%c", green);
        fprintf(file, "%c", blue);
    }

    fclose(file);
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
    rgb_timestamp_ = buffer->timestamp;

    yuv_size_ = yuv_width_ * yuv_height_ + yuv_width_ * yuv_height_ / 2;

    // Reserve and resize the buffer size for RGB and YUV data.
    yuv_buffer_.resize(yuv_size_);
    yuv_buffer_tmp_.resize(yuv_size_);
    rgb_buffer_.resize(yuv_width_ * yuv_height_ * 3);

    std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
    memcpy(&yuv_buffer_tmp_[0], buffer->data, yuv_size_);
    swap_rgb_buffer_signal_ = true;

}


// This function will route callbacks to our application object via the context
// parameter.
// @param context Will be a pointer to a SynchronizationApplication instance  on
// which to call callbacks.
// @param xyz_ij The point cloud to pass on.
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
    point_cloud_timestamp_ = xyz_ij->timestamp;
    callback_point_cloud_buffer_.swap(shared_point_cloud_buffer_);
    swap_point_cloud_buffer_signal_ = true;
  }
}

  SynchronizationApplication::SynchronizationApplication() :
    swap_point_cloud_buffer_signal_(false),
    gpu_upsample_(false),
    swap_yuv_buffer_signal_(false),
    swap_rgb_buffer_signal_(false) {
  // We'll store the fixed transform between the opengl frame convention.
  // (Y-up, X-right) and tango frame convention. (Z-up, X-right).
  OW_T_SS_ = tango_gl::conversions::opengl_world_T_tango_world();
}

SynchronizationApplication::~SynchronizationApplication() {}

int SynchronizationApplication::TangoInitialize(JNIEnv* env,
                                                jobject caller_activity) {
  // The first thing we need to do for any Tango enabled application is to
  // initialize the service. We'll do that here, passing on the JNI environment
  // and jobject corresponding to the Android activity that is calling us.
  return TangoService_initialize(env, caller_activity);
}

int SynchronizationApplication::TangoSetPointCloudRecord(bool isChecked) {
    store_point_clouds_ = isChecked;
    return 1;
}

int SynchronizationApplication::TangoStoreImage(bool store) {
    store_image_ = store;
    return 1;
}

int SynchronizationApplication::TangoSetupConfig() {
  // Here, we'll configure the service to run in the way we'd want. For this
  // application, we'll start from the default configuration
  // (TANGO_CONFIG_DEFAULT). This enables basic motion tracking capabilities.
  // In addition to motion tracking, however, we want to run with depth so that
  // we can sync Image data with Depth Data. As such, we're going to set an
  // additional flag "config_enable_depth" to true.
  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tango_config_ == nullptr) {
    return TANGO_ERROR;
  }

  TangoErrorType ret =
      TangoConfig_setBool(tango_config_, "config_enable_depth", true);
  if (ret != TANGO_SUCCESS) {
    LOGE("Failed to enable depth.");
    return ret;
  }

  // Note that it's super important for AR applications that we enable low
  // latency imu integration so that we have pose information available as
  // quickly as possible. Without setting this flag, you'll often receive
  // invalid poses when calling GetPoseAtTime for an image.
  ret = TangoConfig_setBool(tango_config_,
                            "config_enable_low_latency_imu_integration", true);
  if (ret != TANGO_SUCCESS) {
    LOGE("Failed to enable low latency imu integration.");
    return ret;
  }
  return ret;
}

int SynchronizationApplication::TangoConnectTexture() {
  // The Tango service allows you to connect an OpenGL texture directly to its
  // RGB and fisheye cameras. This is the most efficient way of receiving
  // images from the service because it avoids copies. You get access to the
  // graphic buffer directly. As we're interested in rendering the color image
  // in our render loop, we'll be polling for the color image as needed.
  return TangoService_connectTextureId(
      TANGO_CAMERA_COLOR, color_image_->GetTextureId(), this, nullptr);
}

int SynchronizationApplication::TangoConnectCallbacks() {
  // We're interested in only one callback for this application. We need to be
  // notified when we receive depth information in order to support measuring
  // 3D points. For both pose and color camera information, we'll be polling.
  // The render loop will drive the rate at which we need color images and all
  // our poses will be driven by timestamps. As such, we'll use GetPoseAtTime.
  int ret = TangoService_connectOnXYZijAvailable(OnXYZijAvailableRouter);
  if (ret != TANGO_SUCCESS) {
    LOGE(
            "SynchronizationApplication: Failed to connect to point cloud callback with error"
                    "code: %d",
            ret);
    return ret;
  }

  ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this, OnFrameAvailableRouter);

  if (ret != TANGO_SUCCESS) {
    LOGE(
            "SynchronizationApplication: Failed to connect to on frame available callback with error"
                    "code: %d",
            ret);
    return ret;
  }
}

int SynchronizationApplication::TangoConnect() {
  // Here, we'll connect to the TangoService and set up to run. Note that we're
  // passing in a pointer to ourselves as the context which will be passed back
  // in our callbacks.
  TangoErrorType ret = TangoService_connect(this, tango_config_);
  if (ret != TANGO_SUCCESS) {
    LOGE("SynchronizationApplication: Failed to connect to the Tango service.");
  }
  return ret;
}
int SynchronizationApplication::TangoSetIntrinsicsAndExtrinsics() {
  // Get the intrinsics for the color camera and pass them on to the depth
  // image. We need these to know how to project the point cloud into the color
  // camera frame.
  TangoCameraIntrinsics color_camera_intrinsics;
  TangoErrorType ret = TangoService_getCameraIntrinsics(
      TANGO_CAMERA_COLOR, &color_camera_intrinsics);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Failed to get the intrinsics for the color"
        "camera.");
    return ret;
  }
  depth_image_->SetCameraIntrinsics(color_camera_intrinsics);
  main_scene_->SetCameraIntrinsics(color_camera_intrinsics);

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

  // Pose of device frame wrt imu.
  TangoPoseData pose_imu_T_device;
  // Pose of color frame wrt imu.
  TangoPoseData pose_imu_T_color;
  // Pose of depth camera wrt imu.
  TangoPoseData pose_imu_T_depth;
  TangoCoordinateFramePair frame_pair;
  glm::vec3 translation;
  glm::quat rotation;

  // We need to get the extrinsic transform between the color camera and the
  // imu coordinate frames. This matrix is then used to compute the extrinsic
  // transform between color camera and device:
  // color_T_device = color_T_imu * imu_T_device;
  // Note that the matrix color_T_device is a constant transformation since the
  // hardware will not change, we use the getPoseAtTime() function to query it
  // once right after the Tango Service connected and store it for efficiency.
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
  // Converting pose to transformation matrix.
  glm::mat4 imu_T_device = util::GetMatrixFromPose(&pose_imu_T_device);

  // Get color camera with respect to imu transformation matrix.
  // This matrix is used to compute the extrinsics between color camera and
  // device: color_T_device = color_T_imu * imu_T_device;
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
  // Converting pose to transformation matrix.
  glm::mat4 imu_T_color = util::GetMatrixFromPose(&pose_imu_T_color);

  frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
  frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
  ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_depth);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Failed to get transform between the IMU "
        "and depth camera frames. Something is wrong with device extrinsics.");
    return ret;
  }
  // Converting pose to transformation matrix.
  glm::mat4 imu_T_depth = util::GetMatrixFromPose(&pose_imu_T_depth);

  device_T_color_ = glm::inverse(imu_T_device) * imu_T_color;
  device_T_depth_ = glm::inverse(imu_T_device) * imu_T_depth;

  return ret;
}

void SynchronizationApplication::TangoDisconnect() {
  TangoConfig_free(tango_config_);
  tango_config_ = nullptr;
  TangoService_disconnect();
}

void SynchronizationApplication::InitializeGLContent() {
  depth_image_ = new rgb_depth_sync::DepthImage();
  color_image_ = new rgb_depth_sync::ColorImage();
  main_scene_ = new rgb_depth_sync::Scene(color_image_, depth_image_);
}

void SynchronizationApplication::SetViewPort(int width, int height) {
  screen_width_ = static_cast<float>(width);
  screen_height_ = static_cast<float>(height);
  main_scene_->SetupViewPort(width, height);
}

void SynchronizationApplication::Render() {

  double rgb_timestamp = 0.0;
  double color_timestamp = 0.0;
  double point_cloud_timestamp = 0.0;
  bool new_points = false;

  {
    std::lock_guard<std::mutex> lock(point_cloud_mutex_);
      point_cloud_timestamp = point_cloud_timestamp_;
    if (swap_point_cloud_buffer_signal_) {
        shared_point_cloud_buffer_.swap(render_point_cloud_buffer_);
        swap_point_cloud_buffer_signal_ = false;
        new_points = true;
    }
  }

  {
    std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
    rgb_timestamp = rgb_timestamp_;
    if (swap_rgb_buffer_signal_) {
        yuv_buffer_tmp_.swap(yuv_buffer_);
        swap_rgb_buffer_signal_ = false;
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
  if (TangoService_getPoseAtTime(point_cloud_timestamp, depth_frame_pair,
                                 &pose_start_service_T_device_t0) !=
      TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Could not find a valid pose at time %lf"
        " for the depth camera.",
        point_cloud_timestamp);
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

  // In the following code, we define t0 as the depth timestamp and t1 as the
  // color camera timestamp.
  //
  // Device frame at timestamp t0 (depth timestamp) with respect to start of
  // service.
  glm::mat4 start_service_T_device_t0 =
      util::GetMatrixFromPose(&pose_start_service_T_device_t0);
  // Device frame at timestamp t1 (color timestamp) with respect to start of
  // service.
  glm::mat4 start_service_T_device_t1 =
      util::GetMatrixFromPose(&pose_start_service_T_device_t1);

  // Transformation of depth frame wrt Device at time stamp t0.
  // Transformation of depth frame with respect to the device frame at
  // time stamp t0. This transformation remains constant over time. Here we
  // assign to a local variable to maintain naming consistency when calculating
  // the transform: color_image_t1_T_depth_image_t0.
  glm::mat4 device_t0_T_depth_t0 = device_T_depth_;

  // Transformation of Device Frame wrt Color Image frame at time stamp t1.
  // Transformation of device frame with respect to the color camera frame at
  // time stamp t1. This transformation remains constant over time. Here we
  // assign to a local variable to maintain naming consistency when calculating
  // the transform: color_image_t1_T_depth_image_t0.
  glm::mat4 color_t1_T_device_t1 = glm::inverse(device_T_color_);

  // pose from color camera
  glm::mat4 start_service_T_color_t1 = start_service_T_device_t1 * device_T_color_;

  size_t rgb_image_index = 0;

  if(rgb_timestamp == color_timestamp) {

        rgb_buffer_.resize(yuv_width_ * yuv_height_ * 3);
        rgb_image_buffer_.resize(yuv_width_ * yuv_height_);

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
                Yuv2Rgb(yuv_buffer_[i * yuv_width_ + j],
                        yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index + 1],
                        yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index],
                        &rgb_buffer_[rgb_index],
                        &rgb_buffer_[rgb_index + 1],
                        &rgb_buffer_[rgb_index + 2],
                        &rgb_image_buffer_[i * yuv_width_ + j]);
            }
        }

        if(store_image_){
            std::stringstream ss;
            current_timestamp_ = (int) std::ceil(rgb_timestamp*1000);
            if(current_timestamp_ != previous_timestamp_) {
                ss << current_timestamp_;
                std::string filename = "/storage/sdcard0/Documents/Tango/PPM/" + ss.str() + ".ppm";
                WriteByteToPPM(filename.c_str(), rgb_buffer_, yuv_width_, yuv_height_);
            }
            previous_timestamp_ = current_timestamp_;
            store_image_ = false;
        }
    }


  if (pose_start_service_T_device_t1.status_code == TANGO_POSE_VALID) {
    if (pose_start_service_T_device_t0.status_code == TANGO_POSE_VALID) {
      // Note that we are discarding all invalid poses at the moment, another
      // option could be to use the latest pose when the queried pose is
      // invalid.

      // The Color Camera frame at timestamp t0 with respect to Depth
      // Camera frame at timestamp t1.
      glm::mat4 color_image_t1_T_depth_image_t0 =
          color_t1_T_device_t1 * glm::inverse(start_service_T_device_t1) *
          start_service_T_device_t0 * device_t0_T_depth_t0;

        if(gpu_upsample_) {
            depth_image_->RenderDepthToTexture(color_image_t1_T_depth_image_t0,
                                               render_point_cloud_buffer_, new_points);
        } else {
            depth_image_->UpdateAndUpsampleDepth(color_image_t1_T_depth_image_t0,
                                                 start_service_T_color_t1,
                                                 render_point_cloud_buffer_,
                                                 rgb_image_buffer_,
                                                 color_timestamp,
                                                 store_point_clouds_);
        }
    } else {
      LOGE("Invalid pose for ss_t_depth at time: %lf", point_cloud_timestamp);
    }
  } else {
    LOGE("Invalid pose for ss_t_color at time: %lf", color_timestamp);
  }
  main_scene_->Render();
}

void SynchronizationApplication::FreeGLContent() {
  delete color_image_;
  delete depth_image_;
  delete main_scene_;
}

void SynchronizationApplication::SetDepthAlphaValue(float alpha) {
  main_scene_->SetDepthAlphaValue(alpha);
}

void SynchronizationApplication::SetGPUUpsample(bool on) {
  gpu_upsample_ = on;
}

}  // namespace rgb_depth_sync
