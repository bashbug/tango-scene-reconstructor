#ifndef RGB_DEPTH_SYNC_WRITE_COLOR_IMAGE_H_
#define RGB_DEPTH_SYNC_WRITE_COLOR_IMAGE_H_

namespace rgb_depth_sync {

  class WriteColorImage {
    public:
      WriteColorImage(const char* filename, std::vector<uint8_t> rgb_bytebuffer,
                      size_t w, size_t h);
      ~WriteColorImage(){}
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_WRITE_COLOR_IMAGE_H_