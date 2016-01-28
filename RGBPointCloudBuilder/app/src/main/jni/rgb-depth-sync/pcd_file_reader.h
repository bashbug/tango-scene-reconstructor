//
// Reads a *.PCD file from a given path
// make sure that the file exists before reading from it
// fileformat has to be binary with x,y,z,rgb float values
//

#ifndef RGBPOINTCLOUDBUILDER_PCD_FILE_READER_H
#define RGBPOINTCLOUDBUILDER_PCD_FILE_READER_H

#include <string>
#include <vector>

namespace rgb_depth_sync {

  class PCDFileReader {

    public:
      PCDFileReader();
      ~PCDFileReader();
      void ReadFile(const char* filename);
      std::vector<float> GetTranslation();
      std::vector<float> GetRotation();
      std::vector<float> GetPointsWithRGB();
      std::vector<float> GetPointsWithoutRGB();

    private:
      std::vector<float> translation_;
      std::vector<float> rotation_;
      std::vector<float> points_with_rgb_;
      std::vector<float> points_without_rgb_;
    };

}


#endif //RGBPOINTCLOUDBUILDER_PCD_FILE_READER_H
