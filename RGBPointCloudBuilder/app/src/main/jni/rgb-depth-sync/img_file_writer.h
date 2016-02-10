#ifndef RGB_DEPTH_SYNC_IMG_FILE_WRITER_H_
#define RGB_DEPTH_SYNC_IMG_FILE_WRITER_H_

#include <opencv2/opencv.hpp>

namespace rgb_depth_sync {

  class IMGFileWriter {
    public:
    IMGFileWriter();
    ~IMGFileWriter();
    void SaveToFile(int index, const cv::Mat& img);
    void Save(const char* name, const cv::Mat& img);
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_IMG_FILE_WRITER_H_