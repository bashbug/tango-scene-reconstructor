#include "rgb-depth-sync/img_file_writer.h"

namespace rgb_depth_sync {

  IMGFileWriter::IMGFileWriter() {
  }

  IMGFileWriter::~IMGFileWriter() {
  }

  void IMGFileWriter::SaveToFile(int index, const cv::Mat& img) {
    char filename[1024];
    //LOGE("write file %i start...", pcd_file_counter_);
    sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Img__/%05d.jpg", index);
    cv::imwrite(filename, img);
  }

  void IMGFileWriter::Save(const char* filename, const cv::Mat& img) {
    cv::imwrite(filename, img);
  }
}