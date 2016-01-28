#include "rgb-depth-sync/color_image.h"

namespace rgb_depth_sync {

  ColorImage::ColorImage() {
    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id_);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);

    texture_drawable_ = new rgb_depth_sync::TextureDrawable(shader::kColorCameraVertex,
                                                            shader::kColorCameraFragment);
    color_image_mutex_ = PTHREAD_MUTEX_INITIALIZER;

  }

  void ColorImage::SetImageBuffer(const TangoImageBuffer* buffer) {
    int lock = pthread_mutex_trylock(&color_image_mutex_);
    //color_timestamp = color_timestamp_;
    if(lock == 0) {
      //LOGE("SetImageBuffer locked : %f", buffer->timestamp);
      // only include new images
      std::map<double, cv::Mat>::iterator newest_image = image_buffer_.end();
      if (newest_image->first != buffer->timestamp) {

        cv::Mat yuv_image, rgb_image;
        yuv_image.create(720 * 3 / 2, 1280, CV_8UC1);
        rgb_image.create(720, 1280, CV_8UC3);

        memcpy(yuv_image.data, buffer->data, 1280 * 720 * 3 / 2);

        cv::cvtColor(yuv_image, rgb_image, CV_YUV2RGB_NV21);

        if (image_buffer_.size() >= 10) {
          std::map<double, cv::Mat>::iterator oldest_image = image_buffer_.begin();
          image_buffer_.erase(oldest_image);
        }

        image_buffer_.insert(std::pair<double, cv::Mat>(buffer->timestamp, rgb_image));

        pthread_mutex_unlock(&color_image_mutex_);
      }
    }
  }

  void ColorImage::SetTimestamp(double timestamp) {
    timestamp_ = timestamp;
  }

  void ColorImage::StoreImage(int counter) {
    cv::Mat bgr_image;
    bgr_image.create(720, 1280, CV_8UC3);
    cv::cvtColor(rgb_image_, bgr_image, CV_RGB2BGR);

    char filename[1024];
    sprintf(filename, "/storage/emulated/0/Documents/RGBPointCloudBuilder/Image/%05d.jpg", counter);

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    cv::imwrite(filename, bgr_image, compression_params);
  }

  void ColorImage::GetImage(double depth_timestamp, cv::Mat* requested_image, double* color_timestamp) {
    cv::Mat return_image;
    return_image.create(720, 1240, CV_8UC3);
    double return_timestamp = 0.0;

    std::map<double, cv::Mat>::iterator low, prev;
    low = image_buffer_.lower_bound(depth_timestamp);
    if (low == image_buffer_.end()) {
      --low;
      return_image = low->second;
      return_timestamp = low->first;
    } else if (low == image_buffer_.begin()) {
      return_image = low->second;
      return_timestamp = low->first;
    } else {
      prev = low;
      --prev;
      if (depth_timestamp - prev->first < low->first - depth_timestamp) {
        return_image = prev->second;
        return_timestamp = prev->first;
      } else {
        return_image = low->second;
        return_timestamp = low->first;
      }
    }

    memcpy(requested_image->data, return_image.data, 1280*720*3);
    *color_timestamp = return_timestamp;
  }

  double ColorImage::GetTimestamp() {
    return timestamp_;
  }

  void ColorImage::Draw(int width, int height) {
    if (height == 0 || width == 0) {
      LOGE("The Scene received an invalid height of 0 in SetupViewPort.");
    }
    texture_drawable_->RenderImage(width, height, texture_id_);
  }

  ColorImage::~ColorImage() { glDeleteTextures(1, &texture_id_); }
} // namespace rgb_depth_sync
