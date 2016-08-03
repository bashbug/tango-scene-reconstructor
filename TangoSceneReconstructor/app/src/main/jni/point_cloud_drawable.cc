
/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sstream>

#include "tango-scene-reconstructor/point_cloud_drawable.h"

namespace {
  const std::string kPointCloudVertexShader =
      "precision mediump float;\n"
          "precision mediump int;\n"
          "attribute vec4 vertex;\n"
          "attribute vec3 color;\n"
          "uniform mat4 mvp;\n"
          "varying vec3 v_color;\n"
          "void main() {\n"
          "  gl_PointSize = 4.0;\n"
          "  gl_Position = mvp*vertex;\n"
          "  v_color = color;\n"
          "}\n";
  const std::string kPointCloudFragmentShader =
      "precision mediump float;\n"
          "precision mediump int;\n"
          "varying vec3 v_color;\n"
          "void main() {\n"
          "  gl_FragColor = vec4(vec3(v_color), 1.0);\n"
          "}\n";

  const glm::mat4 kOpengGL_T_Depth =
      glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}  // namespace

namespace tango_scene_reconstructor {

  PointCloudDrawable::PointCloudDrawable() {
    shader_program_ = tango_gl::util::CreateProgram(
        kPointCloudVertexShader.c_str(), kPointCloudFragmentShader.c_str());

    mvp_handle_ = glGetUniformLocation(shader_program_, "mvp");
    vertices_handle_ = glGetAttribLocation(shader_program_, "vertex");
    color_handle_ = glGetAttribLocation(shader_program_, "color");
    glGenBuffers(1, &vertex_buffer_);
    glGenBuffers(1, &color_buffer_);
    glGenBuffers(1, &indices_buffer_);
  }

  void PointCloudDrawable::DeleteGlResources() {
    if (vertex_buffer_) {
      glDeleteBuffers(1, &vertex_buffer_);
      glDeleteBuffers(1, &color_buffer_);
      glDeleteBuffers(1, &indices_buffer_);
    }
    if (shader_program_) {
      glDeleteShader(shader_program_);
    }
  }

  void PointCloudDrawable::Render(glm::mat4 projection_mat,
                           glm::mat4 view_mat,
                           glm::mat4 model_mat,
                           const std::vector<float>& vertices,
                           const std::vector<uint8_t>& rgb_data) {
    glUseProgram(shader_program_);
    mvp_handle_ = glGetUniformLocation(shader_program_, "mvp");

    // Calculate model view projection matrix.
    glm::mat4 mvp_mat = projection_mat * view_mat * model_mat;
    glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));
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

    glDrawArrays(GL_POINTS, 0, vertices.size() / 3);

    glUseProgram(0);
    tango_gl::util::CheckGlError("Pointcloud::Render()");
  }

  void PointCloudDrawable::Render(glm::mat4 projection_mat,
                                  glm::mat4 view_mat,
                                  glm::mat4 model_mat,
                                  const std::vector<float>& vertices,
                                  const std::vector<unsigned int>& indices,
                                  const std::vector<uint8_t>& colors) {

    glUseProgram(shader_program_);
    mvp_handle_ = glGetUniformLocation(shader_program_, "mvp");

    // Calculate model view projection matrix.
    glm::mat4 mvp_mat = projection_mat * view_mat * model_mat;
    glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(vertices_handle_);
    glVertexAttribPointer(vertices_handle_, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, color_buffer_);
    glBufferData(GL_ARRAY_BUFFER, colors.size(), colors.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(color_handle_);
    // index, size, type, stride, normalized, pointer
    glVertexAttribPointer(color_handle_, 3, GL_UNSIGNED_BYTE, GL_TRUE, 0, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GL_UNSIGNED_INT) * indices.size(), &indices[0], GL_STATIC_DRAW);
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, (void*)0);

    glFlush();
    glFinish();
    glUseProgram(0);
    tango_gl::util::CheckGlError("Pointcloud::Render()");
  }

}
