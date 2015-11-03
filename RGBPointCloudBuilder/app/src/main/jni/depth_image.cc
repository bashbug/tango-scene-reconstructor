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


#include <stdio.h>
#include <string.h>
#include <vector>

#include <cmath>
#include <cfenv>
#include <climits>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "tango-gl/conversions.h"
#include "tango-gl/camera.h"

#include "rgb-depth-sync/depth_image.h"

namespace {
const std::string kPointCloudVertexShader =
    "precision mediump float;\n"
    "\n"
    "attribute vec4 vertex;\n"
    "\n"
    "uniform mat4 mvp;\n"
    "uniform float maxdepth;\n"
    "uniform float pointsize;\n"
    "\n"
    "varying vec4 v_color;\n"
    "\n"
    "void main() {\n"
    "  gl_PointSize = pointsize;\n"
    "  gl_Position = mvp*vertex;\n"
    "  float depth = clamp(vertex.z / maxdepth, 0.0, 1.0);\n"
    "  v_color = vec4(depth, depth, depth, 1.0);\n"
    "}\n";
const std::string kPointCloudFragmentShader =
    "precision mediump float;\n"
    "\n"
    "varying vec4 v_color;\n"
    "void main() {\n"
    "  gl_FragColor = v_color;\n"
    "}\n";
}  // namespace

namespace rgb_depth_sync {

DepthImage::DepthImage()
    : texture_id_(0),
      cpu_texture_id_(0),
      gpu_texture_id_(0),
      depth_map_buffer_(0),
      grayscale_display_buffer_(0),
      texture_render_program_(0),
      fbo_handle_(0),
      vertex_buffer_handle_(0),
      vertices_handle_(0),
      mvp_handle_(0) {}

DepthImage::~DepthImage() {
  glDeleteTextures(1, &cpu_texture_id_);
  glDeleteTextures(1, &gpu_texture_id_);

  glDeleteBuffers(1, &vertex_buffer_handle_);
  glDeleteFramebuffers(1, &fbo_handle_);
  glDeleteProgram(texture_render_program_);
}

std::string DepthImage::intToString(int value) {
    std::ostringstream os;
    os << value;
    return os.str();
}

std::string DepthImage::floatToString(float value) {
    std::ostringstream os;
    os << value;
    return os.str();
}

std::string DepthImage::timestampToString(double value) {
    std::ostringstream os;
    value = value * 1000;
    int valueToint = (int) std::ceil(value);
    os << valueToint;
    return os.str();
}

float DepthImage::ConvertINTtoFLOAT(uint32_t significant) {
    // Only support 0 < significant < 1 << 24.
    if (significant == 0 || significant >= 1 << 24)
        return -1.0;  // or abort(); or whatever you'd like here.

    int shifts = 0;

    //  Align the leading 1 of the significant to the hidden-1
    //  position.  Count the number of shifts required.
    while ((significant & (1 << 23)) == 0)
    {
        significant <<= 1;
        shifts++;
    }

    //  The number 1.0 has an exponent of 0, and would need to be
    //  shifted left 23 times.  The number 2.0, however, has an
    //  exponent of 1 and needs to be shifted left only 22 times.
    //  Therefore, the exponent should be (23 - shifts).  IEEE-754
    //  format requires a bias of 127, though, so the exponent field
    //  is given by the following expression:
    unsigned int exponent = 127 + 23 - shifts;

    //  Now merge significand and exponent.  Be sure to strip away
    //  the hidden 1 in the significand.
    unsigned int merged = (exponent << 23) | (significant & 0x7FFFFF);


    //  Reinterpret as a float and return.  This is an evil hack.
    return *reinterpret_cast< float* >( &merged );

}

bool DepthImage::CreateOrBindGPUTexture() {
  if (gpu_texture_id_) {
    glBindTexture(GL_TEXTURE_2D, gpu_texture_id_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_handle_);
    return false;
  } else {
    glGenTextures(1, &gpu_texture_id_);
    texture_render_program_ = tango_gl::util::CreateProgram(
        kPointCloudVertexShader.c_str(), kPointCloudFragmentShader.c_str());

    mvp_handle_ = glGetUniformLocation(texture_render_program_, "mvp");

    glUseProgram(texture_render_program_);
    // Assume these are constant for the life the program
    GLuint max_depth_handle =
        glGetUniformLocation(texture_render_program_, "maxdepth");
    GLuint point_size_handle =
        glGetUniformLocation(texture_render_program_, "pointsize");
    glUniform1f(max_depth_handle,
                static_cast<float>(kMaxDepthDistance) / kMeterToMillimeter);
    glUniform1f(point_size_handle, 2 * kWindowSize + 1);

    vertices_handle_ = glGetAttribLocation(texture_render_program_, "vertex");

    glGenBuffers(1, &vertex_buffer_handle_);

    glBindTexture(GL_TEXTURE_2D, gpu_texture_id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgb_camera_intrinsics_.width,
                 rgb_camera_intrinsics_.height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glGenFramebuffers(1, &fbo_handle_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_handle_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D,
                           gpu_texture_id_, 0);

    return true;
  }
}

bool DepthImage::CreateOrBindCPUTexture() {
  if (cpu_texture_id_) {
    glBindTexture(GL_TEXTURE_2D, cpu_texture_id_);
    return false;
  } else {
    glGenTextures(1, &cpu_texture_id_);
    glBindTexture(GL_TEXTURE_2D, cpu_texture_id_);
    glTexImage2D(GL_TEXTURE_2D, 0,  // mip-map level
                 GL_LUMINANCE, rgb_camera_intrinsics_.width,
                 rgb_camera_intrinsics_.height, 0,  // border
                 GL_LUMINANCE, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    return true;
  }
}

void DepthImage::RenderDepthToTexture(
    glm::mat4& color_t1_T_depth_t0,
    const std::vector<float>& render_point_cloud_buffer, bool new_points) {
  new_points = this->CreateOrBindGPUTexture() || new_points;

  glViewport(0, 0, rgb_camera_intrinsics_.width, rgb_camera_intrinsics_.height);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Special program needed to color by z-distance
  glUseProgram(texture_render_program_);

  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);

  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_handle_);
  if(new_points) {
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * render_point_cloud_buffer.size(),
                 render_point_cloud_buffer.data(), GL_STATIC_DRAW);
  }
  tango_gl::util::CheckGlError("DepthImage Buffer");

  // Skip negation of Y-axis as is normally done in opengl_T_color
  // since we are rendering to a texture that we want to match the Y-axis
  // of the color image.
  const glm::mat4 opengl_T_color(1.0f, 0.0f, 0.0f, 0.0f,
                                 0.0f, 1.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, -1.0f, 0.0f,
                                 0.0f, 0.0f, 0.0f, 1.0f);

  glm::mat4 mvp_mat = projection_matrix_ar_ * opengl_T_color * color_t1_T_depth_t0;

  glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

  glEnableVertexAttribArray(vertices_handle_);
  glVertexAttribPointer(vertices_handle_, 3, GL_FLOAT, GL_FALSE, 0,  nullptr);

  glDrawArrays(GL_POINTS, 0, render_point_cloud_buffer.size() / 3);
  glDisableVertexAttribArray(vertices_handle_);

  tango_gl::util::CheckGlError("DepthImage Draw");

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glUseProgram(0);

  tango_gl::util::CheckGlError("DepthImage RenderTexture");

  texture_id_ = gpu_texture_id_;
}

void DepthImage::SaveOrderedPointCloud(const std::vector<float> point_cloud,
                                       const glm::vec3 translation,
                                       const glm::quat rotation,
                                       int width, int height,
                                       const double timestamp){

    LOGE("WRITE ORDERED POINTCLOUD TO FILE START...");

    /*std::string filename = "/storage/sdcard0/Documents/Tango/PCD/" + timestampToString(timestamp) + ".pcd";
    FILE* file = fopen(filename.c_str(), "wb");

    // write the PCD file header

    fprintf(file, "# .PCD v.7 - Point Cloud Data file format\n");
    fprintf(file, "VERSION .7\n");

    fprintf(file, "FIELDS x y z\n");
    fprintf(file, "SIZE 4 4 4\n");
    fprintf(file, "TYPE F F F\n");
    fprintf(file, "COUNT 1 1 1\n");

    fprintf(file, "WIDTH %d\n", width);
    fprintf(file, "HEIGHT %d\n", height);
    fprintf(file, "VIEWPOINT %lf %lf %lf %lf %lf %lf %lf\n",
            translation[0],
            translation[1],
            translation[2],
            rotation[3],
            rotation[0],
            rotation[1],
            rotation[2]);
    fprintf(file, "POINTS %d\n", point_cloud.size()/3);
    fprintf(file, "DATA binary\n");

    std::vector<float> test;
    int size = point_cloud.size();
    test.resize(size);
    memcpy(&test[0], point_cloud.data(), point_cloud.size());
    fwrite(&test[0], test.size(), sizeof(float), file);

    //fwrite(&point_cloud, sizeof(float), point_cloud.size(), file);
    fclose(file); */

    std::ofstream myfile;
    myfile.open("/storage/sdcard0/Documents/Tango/" +
                timestampToString(timestamp) + ".pcd");
    std::string header = std::string(
            "VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F I\nCOUNT 1 1 1 1\n") +
                         "WIDTH " +  intToString(width) +
                         "\nHEIGHT " + intToString(height)  + "\nVIEWPOINT ";

    // pcl: translation x,y,z rotation w,x,y,z
    // tango: translation x,y,z rotation x,y,z,w
    header += floatToString(translation[0]) + " " + //tx
              floatToString(translation[1]) + " " + //ty
              floatToString(translation[2]) + " " + //tz
              floatToString(rotation[3]) + " " + //rw
              floatToString(rotation[0]) + " " + //rx
              floatToString(rotation[1]) + " " + //ry
              floatToString(rotation[2]); //rz

    std::string header_ascii = header;
    header_ascii += "\nPOINTS " + intToString(point_cloud.size()/4) +
                    "\nDATA ascii\n";
    myfile << header_ascii;

    for(size_t i = 0; i <= point_cloud.size()-4; i=i+4) {
        myfile << floatToString(point_cloud.at(i)) + ' '
                  + floatToString(point_cloud.at(i+1)) + ' '
                  + floatToString(point_cloud.at(i+2)) + ' '
                  + intToString((int)(point_cloud.at(i+3))) + '\n';
    }

    myfile.close();
    LOGE("STOP...");
}

void DepthImage::SaveUnorderedPointCloud(const std::vector<float> point_cloud,
                                         const glm::vec3 translation,
                                         const glm::quat rotation, const double timestamp){

    LOGE("WRITE UNORDERED POINTCLOUD TO FILE START...");

    std::string filename = "/storage/sdcard0/Documents/Tango/PCD/" + timestampToString(timestamp) + ".pcd";
    FILE* file = fopen(filename.c_str(), "wb");

    // write the PCD file header

    fprintf(file, "# .PCD v.7 - Point Cloud Data file format\n");
    fprintf(file, "VERSION .7\n");

    fprintf(file, "FIELDS x y z rgb\n");
    fprintf(file, "SIZE 4 4 4 4\n");
    fprintf(file, "TYPE F F F F\n");
    fprintf(file, "COUNT 1 1 1 1\n");

    fprintf(file, "WIDTH %d\n", point_cloud.size()/4);
    fprintf(file, "HEIGHT %d\n", 1);
    fprintf(file, "VIEWPOINT %lf %lf %lf %lf %lf %lf %lf\n",
            translation[0],
            translation[1],
            translation[2],
            rotation[3],
            rotation[0],
            rotation[1],
            rotation[2]);
    fprintf(file, "POINTS %d\n", point_cloud.size()/4);
    fprintf(file, "DATA binary\n");

    //std::vector<float> test;
    //int size = point_cloud.size();
    //test.resize(size);
    //memcpy(&test[0], point_cloud.data(), point_cloud.size());

    fwrite(&point_cloud[0], point_cloud.size(), sizeof(float), file);

    //fwrite(&point_cloud, sizeof(float), point_cloud.size(), file);
    fclose(file);

    /*std::ofstream myfile;
    myfile.open("/storage/sdcard0/Documents/Tango/" +
                timestampToString(timestamp) + ".pcd");
    std::string header = std::string(
            "VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F I\nCOUNT 1 1 1 1\n") +
                         "WIDTH " +  intToString(point_cloud.size()/4) +
                         "\nHEIGHT 1\nVIEWPOINT ";

    // pcl: translation x,y,z rotation w,x,y,z
    // tango: translation x,y,z rotation x,y,z,w
    header += floatToString(translation[0]) + " " + //tx
              floatToString(translation[1]) + " " + //ty
              floatToString(translation[2]) + " " + //tz
              floatToString(rotation[3]) + " " + //rw
              floatToString(rotation[0]) + " " + //rx
              floatToString(rotation[1]) + " " + //ry
              floatToString(rotation[2]); //rz

    std::string header_ascii = header;
    header_ascii += "\nPOINTS " + intToString(point_cloud.size()/4) +
                    "\nDATA ascii\n";
    myfile << header_ascii;

    for(size_t i = 0; i <= point_cloud.size()-4; i=i+4) {
        myfile << floatToString(point_cloud.at(i)) + ' '
                  + floatToString(point_cloud.at(i+1)) + ' '
                  + floatToString(point_cloud.at(i+2)) + ' '
                  + intToString((int)(point_cloud.at(i+3))) + '\n';
    }

    myfile.close();*/
    /*std::ofstream myfile;
    myfile.open("/storage/sdcard0/Documents/Tango/" +
                timestampToString(timestamp) + ".pcd");
    std::string header = std::string(
            "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n") +
                         "WIDTH " +  intToString(point_cloud.size()/3) +
                         "\nHEIGHT 1\nVIEWPOINT ";

    // pcl: translation x,y,z rotation w,x,y,z
    // tango: translation x,y,z rotation x,y,z,w
    header += floatToString(translation[0]) + " " + //tx
              floatToString(translation[1]) + " " + //ty
              floatToString(translation[2]) + " " + //tz
              floatToString(rotation[3]) + " " + //rw
              floatToString(rotation[0]) + " " + //rx
              floatToString(rotation[1]) + " " + //ry
              floatToString(rotation[2]); //rz

    std::string header_ascii = header;
    header_ascii += "\nPOINTS " + intToString(point_cloud.size()/3) +
                    "\nDATA ascii\n";
    myfile << header_ascii;

    for(unsigned i = 0; i <= point_cloud.size()-3; i=i+3) {
        myfile << floatToString(point_cloud.at(i)) + ' ' + floatToString(point_cloud.at(i+1)) + ' ' + floatToString(point_cloud.at(i+2)) + '\n';
    }

    myfile.close();*/
    LOGE("STOP...");
}

// Update function will be called in application's main render loop. This funct-
// ion takes care of projecting raw depth points on the image plane, and render
// the depth image into a texture.
// This function also takes care of swapping the Render buffer and shared buffer
// if there is new point cloud data available.
// color_t1_T__depth_t0: 't1' represents the color camera frame's timestamp,
// 't0'
// represents
// the depth camera timestamp. The transformation is the depth camera's frame on
// timestamp t0 with respect the rgb camera's frame on timestamp t1.
void DepthImage::UpdateAndUpsampleDepth(
    glm::mat4& color_t1_T_depth_t0,
    glm::mat4& start_service_T_color_t1,
    const std::vector<float> render_point_cloud_buffer,
    const std::vector<uint32_t> render_rgb_image_buffer,
    double timestamp,
    bool store_point_clouds) {

    glm::vec3 translation = GetTranslationFromMatrix(start_service_T_color_t1);
    glm::quat rotation = GetRotationFromMatrix(start_service_T_color_t1);

    int depth_image_width = rgb_camera_intrinsics_.width;
    int depth_image_height = rgb_camera_intrinsics_.height;
    int depth_image_size = depth_image_width * depth_image_height;

    depth_map_buffer_.resize(depth_image_size);
    grayscale_display_buffer_.resize(depth_image_size);
    std::fill(depth_map_buffer_.begin(), depth_map_buffer_.end(), 0.0f);
    std::fill(grayscale_display_buffer_.begin(), grayscale_display_buffer_.end(), 0.0f);

    rgb_display_buffer_.resize(depth_image_size);
    std::fill(rgb_display_buffer_.begin(), rgb_display_buffer_.end(), 0.0f);

    std::vector<float> transformed_point_cloud_to_image_frame;
    std::vector<float> transformed_ordered_point_cloud_to_image_frame;
    transformed_ordered_point_cloud_to_image_frame.resize(depth_image_size*4);
    std::fill(transformed_ordered_point_cloud_to_image_frame.begin(), transformed_ordered_point_cloud_to_image_frame.end(), 0.0f);

    size_t point_cloud_size = render_point_cloud_buffer.size();

    for (size_t i = 0; i <= point_cloud_size-3; i = i+3) {
        float x = render_point_cloud_buffer[i];
        float y = render_point_cloud_buffer[i + 1];
        float z = render_point_cloud_buffer[i + 2];

        // depth_t0_point is the point in depth camera frame on timestamp t0.
        // (depth image timestamp).
        glm::vec4 depth_t0_point = glm::vec4(x, y, z, 1.0);


        // color_t1_point is the point in camera frame on timestamp t1.
        // (color image timestamp).
        glm::vec4 color_t1_point = color_t1_T_depth_t0 * depth_t0_point;

        int pixel_x, pixel_y;
        float rgb_value;

        // get the coordinate on image plane.
        pixel_x = static_cast<int>((rgb_camera_intrinsics_.fx) *
                                   (color_t1_point.x / color_t1_point.z) +
                                   rgb_camera_intrinsics_.cx);

        pixel_y = static_cast<int>((rgb_camera_intrinsics_.fy) *
                                   (color_t1_point.y / color_t1_point.z) +
                                   rgb_camera_intrinsics_.cy);

        if (pixel_x > depth_image_width || pixel_y > depth_image_height || pixel_x < 0 ||
            pixel_y < 0) {
            continue;
        }

        if (store_point_clouds) {
            if (count % 3 == 0) {
                size_t index = pixel_x + pixel_y * depth_image_width;
                if (index >= render_rgb_image_buffer.size()) {
                    LOGE("BAAAAAAAD INDEX: %d, size: %d", index, render_rgb_image_buffer.size());
                } else {
                    uint32_t tmp = render_rgb_image_buffer[index];
                    rgb_value = *reinterpret_cast<float *>(&tmp);
                    /*if (rgb_value == 0.0f) {
                        LOGE("RGB == 0");
                        LOGE("index %zu", index);
                        rgb_value = 16777215;
                    }*/
                    transformed_point_cloud_to_image_frame.push_back(color_t1_point.x);
                    transformed_point_cloud_to_image_frame.push_back(color_t1_point.y);
                    transformed_point_cloud_to_image_frame.push_back(color_t1_point.z);
                    transformed_point_cloud_to_image_frame.push_back(rgb_value);
                }
            }
        }

        // Color value is the GL_LUMINANCE value used for displaying the depth
        // image.
        // We can query for depth value in mm from grayscale image buffer by
        // getting a `pixel_value` at (pixel_x,pixel_y) and calculating
        // pixel_value * (kMaxDepthDistance / USHRT_MAX)
        float depth_value = color_t1_point.z;
        uint8_t grayscale_value =
                (color_t1_point.z * kMeterToMillimeter) * UCHAR_MAX / kMaxDepthDistance;

        UpSampleDepthAroundPoint(rgb_value, grayscale_value, depth_value, pixel_x, pixel_y, &rgb_display_buffer_,
                                 &grayscale_display_buffer_, &depth_map_buffer_);


    }

    float inverseFoclDepthX = 1.0f / rgb_camera_intrinsics_.fx;
    float inverseFoclDepthY = 1.0f / rgb_camera_intrinsics_.fy;

    for (size_t v = 0; v < depth_image_height; v++) {
        for (size_t u = 0; u < depth_image_width; u++) {
            size_t index = v * depth_image_width + u;
            float z = (float) depth_map_buffer_[index];
            float rgb = rgb_display_buffer_[index];

            float point[3];
            // x value
            point[0] = ((float) (u) - rgb_camera_intrinsics_.cx) * z * inverseFoclDepthX;
            // y value
            point[1] = ((float) (v) - rgb_camera_intrinsics_.cy) * z * inverseFoclDepthY;
            // z value
            point[2] = z;

            size_t index2 = (v*depth_image_width + u)*4;
            transformed_ordered_point_cloud_to_image_frame[index2] = point[0];
            transformed_ordered_point_cloud_to_image_frame[index2+1] = point[1];
            transformed_ordered_point_cloud_to_image_frame[index2+2] = point[2];
            transformed_ordered_point_cloud_to_image_frame[index2+3] = rgb;
        }
    }

    if(store_point_clouds) {
        // only every third cloud will be stored
        if (count % 3 == 0) {
            //SaveOrderedPointCloud(transformed_ordered_point_cloud_to_image_frame, translation, rotation, depth_image_width, depth_image_height, timestamp);
            SaveUnorderedPointCloud(transformed_point_cloud_to_image_frame, translation, rotation, timestamp);
        }
        count++;
    }
}

void DepthImage::SetCameraIntrinsics(TangoCameraIntrinsics intrinsics) {
  rgb_camera_intrinsics_ = intrinsics;
  const float kNearClip = 0.1;
  const float kFarClip = 10.0;
  projection_matrix_ar_ = tango_gl::Camera::ProjectionMatrixForCameraIntrinsics(
      intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy,
      intrinsics.cx, intrinsics.cy, kNearClip, kFarClip);
}

void DepthImage::UpSampleDepthAroundPoint(float rgb_value,
    uint8_t grayscale_value, float depth_value, int pixel_x, int pixel_y,
    std::vector<float>* rgb_display_buffer,
    std::vector<uint8_t>* grayscale_buffer,
    std::vector<float>* depth_map_buffer) {

  int image_width = rgb_camera_intrinsics_.width;
  int image_height = rgb_camera_intrinsics_.height;
  int image_size = image_height * image_width;
  // Set the neighbour pixels to same color.
  for (int a = -kWindowSize; a <= kWindowSize; ++a) {
    for (int b = -kWindowSize; b <= kWindowSize; ++b) {
      if (pixel_x > image_width || pixel_y > image_height || pixel_x < 0 ||
          pixel_y < 0) {
        continue;
      }
        size_t pixel_num = (pixel_x + a) + (pixel_y + b) * image_width;

      if (pixel_num > 0 && pixel_num < image_size) {
        (*grayscale_buffer)[pixel_num] = grayscale_value;
        (*depth_map_buffer)[pixel_num] = depth_value;
        (*rgb_display_buffer)[pixel_num] = rgb_value;
      }
    }
  }
}

glm::vec3 DepthImage::GetTranslationFromMatrix(glm::mat4 transformation) {
    glm::vec3 translation;
    glm::quat rotation;
    glm::vec3 scale;

    tango_gl::util::DecomposeMatrix(transformation, translation, rotation, scale);

    return translation;
}

glm::quat DepthImage::GetRotationFromMatrix(glm::mat4 transformation) {
    glm::vec3 translation;
    glm::quat rotation;
    glm::vec3 scale;

    tango_gl::util::DecomposeMatrix(transformation, translation, rotation, scale);

    return  rotation;
}

}  // namespace rgb_depth_sync
