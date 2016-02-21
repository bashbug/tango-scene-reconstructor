#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <tango-gl/conversions.h>
#include "rgb-depth-sync/mesh.h"

namespace rgb_depth_sync {

  Mesh::Mesh(const GLfloat *kVertices, const GLushort *kIndices, const GLfloat *kTextureCoords) {

    glGenBuffers(1, &vertex_buffer_);
    glGenBuffers(1, &color_buffer_);

    glGenBuffers(4, vertex_buffers_);
    tango_gl::util::CheckGlError("Mesh glGenBuffers(3, vertex_buffers_)");
    // Allocate vertices buffer.
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[0]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[0])");
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * 4, kVertices, GL_STATIC_DRAW);
    tango_gl::util::CheckGlError("Mesh glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * 4, kVertices, GL_STATIC_DRAW)");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, 0)");

    // Allocate triangle indices buffer.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertex_buffers_[1]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertex_buffers_[1])");
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * 6, kIndices, GL_STATIC_DRAW);
    tango_gl::util::CheckGlError("Mesh glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort) * 6, kIndices, GL_STATIC_DRAW)");
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)");

    // Allocate texture coordinates buffer.
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[2]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[2])");
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 4, kTextureCoords, GL_STATIC_DRAW);
    tango_gl::util::CheckGlError("Mesh glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * 4, kTextureCoords, GL_STATIC_DRAW)");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, 0)");

    // Allocate rgb color buffer.
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[3]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[3])");
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLushort) * 3 * 4, kVertices, GL_STATIC_DRAW);
    tango_gl::util::CheckGlError("Mesh glBufferData(GL_ARRAY_BUFFER, sizeof(GLushort) * 3 * 4, kVertices, GL_STATIC_DRAW)");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, 0)");

    // Assign the vertices attribute data.
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[0]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[0])");
    glEnableVertexAttribArray(0);
    tango_gl::util::CheckGlError("Mesh glEnableVertexAttribArray(0)");
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    tango_gl::util::CheckGlError("Mesh glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr)");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, 0)");

    // Assign the texture coordinates attribute data.
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[2]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[2])");
    glEnableVertexAttribArray(1);
    tango_gl::util::CheckGlError("Mesh glEnableVertexAttribArray(1)");
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    tango_gl::util::CheckGlError("Mesh glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr)");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, 0)");

    // Assign the color attribute data.
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[3]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[3])");
    glEnableVertexAttribArray(2);
    tango_gl::util::CheckGlError("Mesh glEnableVertexAttribArray(2)");
    glVertexAttribPointer(2, 3, GL_UNSIGNED_SHORT, GL_FALSE, 0, nullptr);
    tango_gl::util::CheckGlError("Mesh glVertexAttribPointer(2, 3, GL_UNSIGNED_SHORT, GL_FALSE, 0, nullptr)");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, 0)");
  }

  Mesh::~Mesh() { }

  void Mesh::Draw() {
    glDisable(GL_DEPTH_TEST);
    tango_gl::util::CheckGlError("Mesh glDisable(GL_DEPTH_TEST)");

    // Bind vertices buffer to id 0 of shader program
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[0]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[0])");
    glEnableVertexAttribArray(0);
    tango_gl::util::CheckGlError("Mesh glEnableVertexAttribArray(0)");
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    tango_gl::util::CheckGlError("Mesh glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr)");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, 0)");

    // Bind texture coordinates buffer to id 1 of shader program
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[2]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_[2])");
    glEnableVertexAttribArray(1);
    tango_gl::util::CheckGlError("Mesh glEnableVertexAttribArray(1)");
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    tango_gl::util::CheckGlError("Mesh glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr)");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ARRAY_BUFFER, 0)");

    // Bind element array buffer.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertex_buffers_[1]);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertex_buffers_[1])");

    // Draw vertices with 6 indices
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);
    tango_gl::util::CheckGlError("Mesh glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0)");

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    tango_gl::util::CheckGlError("Mesh glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)");
  }

  void Mesh::DrawPointCloud(glm::mat4 projection_mat, glm::mat4 view_mat, glm::mat4 model_mat,
                            const std::vector<float>& vertices,
                            const std::vector<uint8_t>& rgb_data,
                            GLuint shader_id) {

    //glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    GLuint mvp_id = glGetUniformLocation(shader_id, "mvp");
    tango_gl::util::CheckGlError("Mesh glEnable(GL_VERTEX_PROGRAM_POINT_SIZE)");
    // Calculate model view projection matrix.
    glm::mat4 mvp_mat = projection_mat * view_mat * model_mat;
    tango_gl::util::CheckGlError("Mesh glm::mat4 mvp_mat = projection_mat * ...");
    glUniformMatrix4fv(mvp_id, 1, GL_FALSE, glm::value_ptr(mvp_mat));
    tango_gl::util::CheckGlError("Mesh glUniformMatrix4fv(mvp_id, 1, GL_FALSE, glm::value_ptr(mvp_mat))");

    vertices_handle_ = glGetAttribLocation(shader_id, "vertex");
    color_handle_ = glGetAttribLocation(shader_id, "color");

    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(vertices_handle_);
    glVertexAttribPointer(vertices_handle_, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, color_buffer_);
    glBufferData(GL_ARRAY_BUFFER, rgb_data.size(), rgb_data.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(color_handle_);
                          // index, size, type, stride, normalized, pointer
    glVertexAttribPointer(color_handle_, 3, GL_UNSIGNED_BYTE, GL_TRUE, 0, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glDrawArrays(GL_POINTS, 0, vertices.size()/3);

    glUseProgram(0);
    tango_gl::util::CheckGlError("Mesh glDrawArrays(GL_POINTS, 0, vertices.size() / 3)");
  }
}