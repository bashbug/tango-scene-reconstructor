/**
 * Helper class writes a *.PCD file to
 * Documents/RGBPointCloudBuilder/yyyyMMddHHmmss/PCD
 * make sure that this folder exists before writing to it
 * The fileformat is binary with x,y,z,rgb float values
 */

#ifndef RGBPOINTCLOUDBUILDER_PCD_FILE_WRITER_H
#define RGBPOINTCLOUDBUILDER_PCD_FILE_WRITER_H

#include <string>
#include <vector>
#include <tango-gl/util.h>

#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  class PCDFileWriter {

    public:
      PCDFileWriter();
      ~PCDFileWriter();
      void SetUnordered();
      void SaveToFile(const char* dest_folder, int num);
      void SetPCDRGBData(const std::vector<float> pcd,
                         const std::vector<float> translation,
                         const std::vector<float> rotation);
      void SetPCDRGBData(const std::vector<float> pcd,
                         const glm::vec3 translation,
                         const glm::quat rotation);
      void SetPCDData(const std::vector<float> pcd,
                      const std::vector<float> translation,
                      const std::vector<float> rotation);
      void SetPCDData(const std::vector<float> pcd,
                      const glm::vec3 translation,
                      const glm::quat rotation);
      void SetHeader(int width, int height);
      void SetRGBHeader(int width, int height);
      std::string GetHeader();
      std::string GetRGBHeader();

    private:
      std::string rgb_header_;
      std::string header_;
      std::vector <float> pcd_;
      std::vector <float> vertices_;
      std::vector <uint8_t> rgb_values_;
      float translation_[3];
      float rotation_[4];
      size_t pcd_file_counter_ = 1;
      std::vector <uint8_t> rgb_data_;
      bool with_rgb_;
  };
}


#endif //RGBPOINTCLOUDBUILDER_PCD_FILE_WRITER_H
