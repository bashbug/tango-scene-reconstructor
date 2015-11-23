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

  void PointCloudData::setUnordered() {
    setHeader(pcd_.size()/4, 1);
  }

  void PointCloudData::setOrdered(int width, int height) {
    setHeader(width, height);
  }

  void PointCloudData::saveToFile() {

    LOGE("WRITE UNORDERED POINTCLOUD TO FILE START...");

    std::string filename = "/storage/emulated/0/Documents/RGBPointCloudBuilder/PCD/" + timestamp_ + ".pcd";
    FILE *file = fopen(filename.c_str(), "wb");

    // write the PCD file header
    fprintf(file, "%s", getHeader().c_str());

    // write rgb point cloud buffer
    fwrite(&pcd_[0], pcd_.size(), sizeof(float), file);
    fclose(file);

    LOGE("STOP...");
  }

  void PointCloudData::saveToSocket(std::string addr, int port) {
    if(pcd_.size() > 0) {
      tcp_client* c = new rgb_depth_sync::tcp_client(addr, port);
      // send filename which is the point cloud tango timestamp
      char *filename = new char[timestamp_.length() + 1];
      strcpy(filename, timestamp_.c_str());
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
    }
  }

  std::vector<float> PointCloudData::getPCDData() {
    return pcd_;
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

} // namespace rgb_depth_sync