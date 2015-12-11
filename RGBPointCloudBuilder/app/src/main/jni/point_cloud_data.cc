#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "rgb-depth-sync/point_cloud_data.h"
#include "rgb-depth-sync/tcp_client.h"

namespace rgb_depth_sync {

  PointCloudData::PointCloudData() { }

  void PointCloudData::setPCDData(const std::vector <float> pcd,
                                  const glm::vec3 translation, const glm::quat rotation,
                                  const double timestamp) {
    timestamp_ = timestampToString(timestamp);
    translation_ = translation;
    rotation_ = rotation;
    pcd_ = pcd;
  }

  void PointCloudData::SetRGBDData(glm::mat4 &color_image_t1_T_depth_image_t0,
                                    glm::mat4 &open_gl_T_ss_T_color_T_depth,
                                    const std::vector <float> &render_point_cloud_buffer,
                                    const std::vector <uint8_t> &rgb_map_buffer) {

    int depth_image_width = rgb_camera_intrinsics_.width;
    int depth_image_height = rgb_camera_intrinsics_.height;
    int depth_image_size = depth_image_width * depth_image_height;

    size_t point_cloud_size = render_point_cloud_buffer.size();


    if(&(render_point_cloud_buffer) != nullptr && &(rgb_map_buffer) != nullptr) {
      transformed_unordered_point_cloud_to_image_frame_.clear();
      rgb_data_.clear();
      LOGE("size rgb: %i", rgb_map_buffer.size());
      LOGE("size pcd: %i", render_point_cloud_buffer.size());

      for (int i = 0; i < render_point_cloud_buffer.size() - 3; i = i + 3) {
        float x = render_point_cloud_buffer[i];
        float y = render_point_cloud_buffer[i + 1];
        float z = render_point_cloud_buffer[i + 2];

        // depth_t0_point is the point in depth camera frame on timestamp t0.
        // (depth image timestamp).
        glm::vec4 depth_t0_point = glm::vec4(x, y, z, 1.0);

        // color_t1_point is the point in camera frame on timestamp t1.
        // (color image timestamp).
        glm::vec4 color_point = color_image_t1_T_depth_image_t0 * depth_t0_point;
        glm::vec4 opengl_point = open_gl_T_ss_T_color_T_depth * depth_t0_point;

        int pixel_x, pixel_y;
        // get the coordinate on image plane.
        pixel_x = static_cast<int>((rgb_camera_intrinsics_.fx) *
                                   (color_point.x / color_point.z) +
                                   rgb_camera_intrinsics_.cx);

        pixel_y = static_cast<int>((rgb_camera_intrinsics_.fy) *
                                   (color_point.y / color_point.z) +
                                   rgb_camera_intrinsics_.cy);

        if (pixel_x > depth_image_width || pixel_y > depth_image_height || pixel_x < 0 ||
            pixel_y < 0) {
          continue;
        }

        size_t index = (pixel_x + pixel_y * rgb_camera_intrinsics_.width) * 3;

        transformed_unordered_point_cloud_to_image_frame_.push_back(opengl_point.x);
        transformed_unordered_point_cloud_to_image_frame_.push_back(opengl_point.y);
        transformed_unordered_point_cloud_to_image_frame_.push_back(opengl_point.z);

        if(index >= 0 || index+2 < rgb_map_buffer.size()) {
          rgb_data_.push_back(rgb_map_buffer[index]);
          rgb_data_.push_back(rgb_map_buffer[index+1]);
          rgb_data_.push_back(rgb_map_buffer[index+2]);
        } else {
          LOGE("evil index: %i", index);
        }
      }
      LOGE("transfromt_pcd size: %i", transformed_unordered_point_cloud_to_image_frame_.size());
      LOGE("transformt_rgb size: %i", rgb_data_.size());
    }

  }

  void PointCloudData::setUnordered() {
    setHeader(pcd_.size()/4, 1);
  }

  void PointCloudData::setOrdered(int width, int height) {
    setHeader(width, height);
  }

  void PointCloudData::saveToFile() {

    LOGE("WRITE UNORDERED POINTCLOUD TO FILE START...");

    char filename[1024];
    sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/%05d.pcd", pcd_file_counter_);

    FILE *file = fopen(filename, "wb");

    // write the PCD file header
    fprintf(file, "%s", getHeader().c_str());

    // write rgb point cloud buffer
    fwrite(&pcd_[0], pcd_.size(), sizeof(float), file);
    fclose(file);

    pcd_file_counter_++;

    LOGE("STOP...");
  }

  void PointCloudData::saveToSocket(std::string addr, int port) {
    if(pcd_.size() > 0) {
      tcp_client* c = new rgb_depth_sync::tcp_client(addr, port);
      // send filename which is the point cloud tango timestamp

      char filename[1024];
      sprintf(filename, "%05d", pcd_file_counter_);

      //char* filename = "test.txt";
      long long filenamesize = strlen(filename);

      // send header + point cloud data
      std::string header = getHeader();
      char *cstr = new char[header.length() + 1];
      strcpy(cstr, header.c_str());

      long long datasize = strlen(cstr) + (pcd_.size() * sizeof(float));

      c->sendlong(filenamesize);
      c->sendlong(datasize);
      c->sendfilename(filename);
      c->sendpcddata(cstr, pcd_);
      delete [] filename;
      delete [] cstr;

      pcd_file_counter_++;
    }
  }

  std::vector<float> PointCloudData::getPCDData() {
    return pcd_;
  }

  std::vector<float>PointCloudData::GetRGBDData() {
    return transformed_unordered_point_cloud_to_image_frame_;
  }

  std::vector<uint8_t> PointCloudData::GetRGBData() {
    return rgb_data_;
  }

  std::string PointCloudData::getHeader() {
    return header_;
  }

  void PointCloudData::setHeader(int width, int height) {
    std::stringstream header;
    header << "# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
    << "WIDTH " << width << "\nHEIGHT " << height << "\n"
    << "VIEWPOINT "
    << translation_[0] << " "
    << translation_[1] << " "
    << translation_[2] << " "
    << rotation_[3] << " "
    << rotation_[0] << " "
    << rotation_[1] << " "
    << rotation_[2] << "\n"
    << "POINTS " << (pcd_.size() / 4) << "\n"
    << "DATA binary\n";

    header_ = header.str();
  }

  std::string PointCloudData::timestampToString(double value) {
    std::ostringstream os;
    value = value * 1000;
    int valueToint = (int) std::ceil(value);
    os << valueToint;
    return os.str();
  }

  void PointCloudData::SetCameraIntrinsics(TangoCameraIntrinsics intrinsics) {
    rgb_camera_intrinsics_ = intrinsics;
    /*const float kNearClip = 0.1;
    const float kFarClip = 10.0;
    projection_matrix_ar_ = tango_gl::Camera::ProjectionMatrixForCameraIntrinsics(
        intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy,
        intrinsics.cx, intrinsics.cy, kNearClip, kFarClip);*/
  }

} // namespace rgb_depth_sync