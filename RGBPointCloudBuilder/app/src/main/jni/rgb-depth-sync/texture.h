/*
 * Provides textures for rgb, gray or color camera images
 */

#ifndef RGB_DEPTH_SYNC_TEXTURE_H
#define RGB_DEPTH_SYNC_TEXTURE_H

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include "tango-gl/util.h"

namespace rgb_depth_sync {

  class Texture {
    public:
      Texture();
      ~Texture();
      void SetTextureRGBData(void* data, int width, int height);
      void SetTextureGrayData(void* data, int width, int height);
      void Bind(GLuint shader_id);
      void Bind(GLuint shader_id, GLuint texture_id);
    private:
      GLuint texture_id_;
  };

}

#endif //RGB_DEPTH_SYNC_TEXTURE_H
