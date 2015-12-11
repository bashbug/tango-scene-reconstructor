/*
 * Class to setup a vertices array for rendering
 */

#ifndef RGB_DEPTH_SYNC_MESH_H_
#define RGB_DEPTH_SYNC_MESH_H_

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include "tango-gl/util.h"

namespace {
  static const glm::mat4 kOpengGL_T_Depth =
      glm::mat4(1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, -1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, -1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f);
} // namespace

namespace rgb_depth_sync {

  class Mesh {
    public:
      Mesh(const GLfloat *kVertices, const GLushort *kIndices,
           const GLfloat *kTextureCoords);
      ~Mesh();
      void Draw();
      void DrawPointCloud(glm::mat4 projection_mat, glm::mat4 view_mat, glm::mat4 model_mat,
                          const std::vector<float>& vertices,
                          const std::vector<uint8_t>& rgb_data,
                          GLuint shader_id);

      private:
      GLuint vertex_buffers_[4];
      GLuint vertex_buffer_;
      GLuint color_buffer_;
      GLuint vertices_handle_;
      GLuint color_handle_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_MESH_H_