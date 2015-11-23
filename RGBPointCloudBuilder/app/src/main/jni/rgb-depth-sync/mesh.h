/*
 * Class to setup a vertices array for rendering
 */

#ifndef RGB_DEPTH_SYNC_MESH_H_
#define RGB_DEPTH_SYNC_MESH_H_

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include "tango-gl/util.h"

namespace rgb_depth_sync {

  class Mesh {
    public:
      Mesh(const GLfloat *kVertices, const GLushort *kIndices,
           const GLfloat *kTextureCoords);
      ~Mesh();
      void Draw();
      void DrawFrameBuffer(void *data, size_t size);
    private:
      GLuint vertex_buffers_[3];
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_MESH_H_