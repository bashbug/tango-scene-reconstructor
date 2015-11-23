#include <vector>
#include <stdlib.h>
#include <stdio.h>

#include "rgb-depth-sync/write_color_image.h"

namespace rgb_depth_sync {

  WriteColorImage::WriteColorImage(const char* filename, std::vector<uint8_t> rgb_bytebuffer,
                                   size_t w, size_t h) {

    FILE *file = fopen(filename, "wb");
    // ppm header
    fprintf(file, "P6\n%zu %zu\n255\n", w, h);
    // image data as unsigned char r,g,b values
    fwrite(&rgb_bytebuffer[0], rgb_bytebuffer.size(), sizeof(uint8_t), file);
    fclose(file);
  }
}