/*
 * Color image includes conversion from yuv to rgb
 * storing in "/storage/emulated/0/Documents/RGBPointCloudBuilder/Image/
 * make sure that this folder exists
 * and drawing it as an OpenGl texture
 */

#ifndef RGB_DEPTH_SYNC_COLOR_IMAGE_H_
#define RGB_DEPTH_SYNC_COLOR_IMAGE_H_

#include <thread>
#include <mutex>
#include <errno.h>
#include <pthread.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <tango-gl/util.h>
#include <tango_client_api.h>
#include <opencv2/opencv.hpp>

#include "rgb-depth-sync/shader.h"
#include "rgb-depth-sync/texture_drawable.h"

#define MAX_IMAGE_FRAME     10
#define COLOR_IMAGE_WIDTH   1280
#define COLOR_IMAGE_HEIGHT  720

namespace rgb_depth_sync {

  class ColorImage {
    public:
      ColorImage();
      ~ColorImage();
      void SetImageBuffer(const TangoImageBuffer* buffer);
      void SetTimestamp(double timestamp);
      void GetImage(double timestamp, cv::Mat* requested_image, double* color_timestamp);
      double GetTimestamp();
      void StoreImage(int counter);
      void Draw(int width, int height);
      GLuint GetTextureId() const {return texture_id_;}
      pthread_mutex_t color_image_mutex_;
  private:
      double timestamp_;
      cv::Mat rgb_image_;
      cv::Mat yuv_image_;
      int index_;
      TextureDrawable* texture_drawable_;
      GLuint texture_id_;
      std::map<double, cv::Mat> image_buffer_;

  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_COLOR_IMAGE_H_
