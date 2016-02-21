#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "rgb-depth-sync/pcd_file_writer.h"
#include "rgb-depth-sync/tcp_client.h"
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  PCDFileWriter::PCDFileWriter() { }
  PCDFileWriter::~PCDFileWriter() { }

  void PCDFileWriter::SetPCDRGBData(const std::vector<float> pcd,
                                    const std::vector<float> translation,
                                    const std::vector<float> rotation) {
    translation_[0] = translation[0];
    translation_[1] = translation[1];
    translation_[2] = translation[2];
    rotation_[0] = rotation[0];
    rotation_[1] = rotation[1];
    rotation_[2] = rotation[2];
    rotation_[3] = rotation[3];
    pcd_ = pcd;
    with_rgb_ = true;
  }

  void PCDFileWriter::SetPCDRGBData(const std::vector<float> pcd,
                                    const glm::vec3 translation,
                                    const glm::quat rotation) {
    translation_[0] = translation[0];
    translation_[1] = translation[1];
    translation_[2] = translation[2];
    rotation_[0] = rotation.w;
    rotation_[1] = rotation.x;
    rotation_[2] = rotation.y;
    rotation_[3] = rotation.z;
    pcd_ = pcd;
    with_rgb_ = true;
  }

  void PCDFileWriter::SetPCDData(const std::vector<float> pcd,
                                 const std::vector<float> translation,
                                 const std::vector<float> rotation) {
    translation_[0] = translation[0];
    translation_[1] = translation[1];
    translation_[2] = translation[2];
    rotation_[0] = rotation[0];
    rotation_[1] = rotation[1];
    rotation_[2] = rotation[2];
    rotation_[3] = rotation[3];
    pcd_ = pcd;
    with_rgb_ = false;
  }

  void PCDFileWriter::SetPCDData(const std::vector<float> pcd,
                                 const glm::vec3 translation,
                                 const glm::quat rotation) {
    translation_[0] = translation[0];
    translation_[1] = translation[1];
    translation_[2] = translation[2];
    rotation_[0] = rotation.w;
    rotation_[1] = rotation.x;
    rotation_[2] = rotation.y;
    rotation_[3] = rotation.z;
    pcd_ = pcd;
    with_rgb_ = false;
  }

  void PCDFileWriter::SaveToFile(std::string dest_folder, int num) {
    char filename[1024];
    //LOGE("write file %i start...", pcd_file_counter_);
    sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/%s/%05d.pcd", dest_folder.c_str(),
            num);

    FILE *file = fopen(filename, "wb");

    // write the PCD file header
    if (with_rgb_) {
      fprintf(file, "%s", GetRGBHeader().c_str());
    } else {
      fprintf(file, "%s", GetHeader().c_str());
    }

    // write rgb point cloud buffer
    fwrite(&pcd_[0], pcd_.size(), sizeof(float), file);
    fclose(file);
  }

  void PCDFileWriter::SaveToSocket(std::string addr, int port) {
    if (pcd_.size() > 0) {
      tcp_client *c = new rgb_depth_sync::tcp_client(addr, port);

      char filename[1024];
      sprintf(filename, "%05d", pcd_file_counter_);

      //char* filename = "test.txt";
      long long filenamesize = strlen(filename);

      // send header + point cloud data
      std::string header;
      if(with_rgb_) {
        header = GetRGBHeader();
      } else {
        header = GetHeader();
      }
      char *cstr = new char[header.length() + 1];
      strcpy(cstr, header.c_str());

      long long datasize = strlen(cstr) + (pcd_.size() * sizeof(float));

      c->sendlong(filenamesize);
      c->sendlong(datasize);
      c->sendfilename(filename);
      c->sendpcddata(cstr, pcd_);

      pcd_file_counter_++;
    }
  }

  void PCDFileWriter::SetUnordered() {
    if (with_rgb_) {
      SetRGBHeader(pcd_.size() / 4, 1);
    } else {
      SetHeader(pcd_.size() / 3, 1);
    }
  }

  void PCDFileWriter::SetOrdered(int width, int height) {
    if (with_rgb_) {
      SetRGBHeader(width, height);
    } else {
      SetHeader(width, height);
    }
  }

  void PCDFileWriter::SetHeader(int width, int height) {
    std::stringstream header;
    header <<
    "# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
    << "WIDTH " << width << "\nHEIGHT " << height << "\n"
    << "VIEWPOINT "
    << translation_[0] << " "
    << translation_[1] << " "
    << translation_[2] << " "
    << rotation_[0] << " " // w
    << rotation_[1] << " " // x
    << rotation_[2] << " " // y
    << rotation_[3] << "\n" // z
    << "POINTS " << (pcd_.size() / 3) << "\n"
    << "DATA binary\n";

    header_ = header.str();
  }

  void PCDFileWriter::SetRGBHeader(int width, int height) {
    std::stringstream header;
    header <<
    "# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
    << "WIDTH " << width << "\nHEIGHT " << height << "\n"
    << "VIEWPOINT "
    << translation_[0] << " "
    << translation_[1] << " "
    << translation_[2] << " "
    << rotation_[0] << " " // w
    << rotation_[1] << " " // x
    << rotation_[2] << " " // y
    << rotation_[3] << "\n" // z
    << "POINTS " << (pcd_.size() / 4) << "\n"
    << "DATA binary\n";

    rgb_header_ = header.str();
  }

  std::string PCDFileWriter::GetHeader() {
    return header_;
  }

  std::string PCDFileWriter::GetRGBHeader() {
    return rgb_header_;
  }
}