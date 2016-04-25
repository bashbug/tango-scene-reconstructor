#include "rgb-depth-sync/pcd_file_reader.h"
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  PCDFileReader::PCDFileReader() {}
  PCDFileReader::~PCDFileReader() {
    LOGE("PCDFileReader: I get deleted");
  }

  void PCDFileReader::ReadFile(const char* filename) {

      std::ifstream fs;
      std::string line;

      unsigned int data_idx;

      fs.open(filename, std::ios::binary);

      // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
      // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
      std::vector<int> field_sizes, field_counts;
      // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
      std::vector<char> field_types;
      std::vector<std::string> st;

      translation_.resize(3);
      rotation_.resize(4);

      unsigned int width;

      while(!fs.eof()) {

        getline(fs, line);


        // Ignore empty lines
        if (line == "")
          continue;

        std::stringstream sstream (line);
        sstream.imbue (std::locale::classic());

        std::string line_type;
        sstream >> line_type;

        // Ignore comments
        if (line_type.substr (0, 1) == "#")
          continue;

        // Version numbers are not needed for now, but we are checking to see if they're there
        if (line_type.substr (0, 7) == "VERSION")
          continue;

        if (line_type.substr (0, 6) == "FIELDS")
          continue;

        if (line_type.substr (0, 4) == "SIZE")
          continue;

        if (line_type.substr (0, 4) == "TYPE")
          continue;

        if (line_type.substr (0, 5) == "COUNT")
          continue;

        if (line_type.substr (0, 5) == "WIDTH") {
          sstream >> width;
          continue;
        }

        if (line_type.substr (0, 6) == "HEIGHT")
          continue;

        if (line_type.substr (0, 9) == "VIEWPOINT") {
          float x, y, z, w;

          sstream >> x >> y >> z ;
          translation_[0] = x;
          translation_[1] = y;
          translation_[2] = z;

          sstream >> w >> x >> y >> z;
          rotation_[0] = w;
          rotation_[1] = x;
          rotation_[2] = y;
          rotation_[3] = z;

          continue;
        }

        if (line_type.substr (0, 6) == "POINTS")
          continue;

        if (line_type.substr (0, 4) == "DATA") {
          data_idx = static_cast<int>(fs.tellg());
            continue;
        }

        unsigned int data_size = data_idx + width*4*sizeof(float);

        fs.close();

        int fd = open(filename, O_RDONLY);

        // store the complete file in a char buffer
        char *map = static_cast<char*> (mmap(0, data_size, PROT_READ, MAP_SHARED, fd, 0));

        std::vector<float> tmp_buffer;
        tmp_buffer.resize(width*4);

        // copy the relevant data (starting from data_idx) of the char file buffer into the point buffer
        memcpy(&tmp_buffer[0], &map[0] + data_idx, width*4*sizeof(float));

        points_with_rgb_.clear();
        points_without_rgb_.clear();

        points_with_rgb_ = tmp_buffer;

        // without rgb values
        for (int i = 0; i < points_with_rgb_.size(); i=i+4) {
          points_without_rgb_.push_back(points_with_rgb_[i]);
          points_without_rgb_.push_back(points_with_rgb_[i+1]);
          points_without_rgb_.push_back(points_with_rgb_[i+2]);
        }

        close(fd);
      }
    }

    std::vector<float> PCDFileReader::GetTranslation() {
      return translation_;
    }

    std::vector<float> PCDFileReader::GetRotation() {
      return rotation_;
    }

    std::vector<float> PCDFileReader::GetPointsWithRGB() {
      return points_with_rgb_;
    }

    std::vector<float> PCDFileReader::GetPointsWithoutRGB() {
      return points_without_rgb_;
    }
}

