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

#include "rgb-depth-sync/texture_drawable.h"
#include "rgb-depth-sync/texture.h"
#include "rgb-depth-sync/shader.h"
#include "rgb-depth-sync/mesh.h"

namespace {
  // positions of the given vertices, it will fill up the screen with the given texture
  const GLfloat kVertices[] = {-1.0,1.0, 0.0,-1.0, -1.0,0.0,
                                1.0,1.0, 0.0, 1.0, -1.0,0.0};

  // for optimization we do not want to duplicate vertices over and over again, reference it with an
  // index just once
  const GLushort kIndices[] = {0, 1, 2, 2, 1, 3};

  // each vertex of the given vertex array above has an texture coordinate.
  // The complete texture will fill out the screen
  const GLfloat kTextureCoords[] = {0.0, 0.0,
                                    0.0, 1.0,
                                    1.0, 0.0,
                                    1.0, 1.0};
}  // namespace

namespace rgb_depth_sync {

  TextureDrawable::TextureDrawable(const char* vertex_source,
                                   const char* fragment_source) {
    shader_ = new rgb_depth_sync::Shader();
    shader_id_ = shader_->CreateProgram(vertex_source, fragment_source);
    if (!shader_id_) {
      LOGE("Could not create shader program for RangeImageTextureDrawable.");
    }
    texture_ = new rgb_depth_sync::Texture();
    mesh_ = new rgb_depth_sync::Mesh(kVertices, kIndices, kTextureCoords);
  }

  TextureDrawable::~TextureDrawable() {
    delete shader_;
  }

  void TextureDrawable::SetGrayTexture(void* data, int width, int height) {
    texture_->SetTextureGrayData(data, width, height);
  }

  void TextureDrawable::SetRGBTexture(void* data, int width, int height) {
    texture_->SetTextureRGBData(data, width, height);
  }

  void TextureDrawable::RenderImage(int width, int height) {
    glEnable(GL_DEPTH_TEST);
    tango_gl::util::CheckGlError("RangeImage glEnable(GL_DEPTH_TEST)");
    glEnable(GL_CULL_FACE);
    tango_gl::util::CheckGlError("RangeImage glEnable(GL_CULL_FACE)");
    glViewport(0, 0, width, height);
    tango_gl::util::CheckGlError("RangeImage glViewport(0, 0, width, height)");
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    tango_gl::util::CheckGlError("RangeImage glClearColor(0.0f, 0.0f, 0.0f, 1.0f)");
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    tango_gl::util::CheckGlError("RangeImage glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)");

    shader_->Bind();
    texture_->Bind(shader_id_);
    //mesh_->Draw();
    tango_gl::util::CheckGlError("RangeImageTextureDrawable::render");
    glUseProgram(0);
  }

  void TextureDrawable::RenderImage(int width, int height, GLuint texture_id) {
    glEnable(GL_DEPTH_TEST);
    tango_gl::util::CheckGlError("ColorImage glEnable(GL_DEPTH_TEST)");
    glEnable(GL_CULL_FACE);
    tango_gl::util::CheckGlError("ColorImage glEnable(GL_CULL_FACE)");
    glViewport(0, 0, width, height);
    tango_gl::util::CheckGlError("ColorImage glViewport(0, 0, width, height)");
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    tango_gl::util::CheckGlError("ColorImage glClearColor(0.0f, 0.0f, 0.0f, 1.0f)");
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    tango_gl::util::CheckGlError("ColorImage glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)");

    shader_->Bind();
    texture_->Bind(shader_id_, texture_id);
    //mesh_->Draw();
    tango_gl::util::CheckGlError("RangeImageTextureDrawable::render");
    glUseProgram(0);
  }

  void TextureDrawable::RenderPointCloud(glm::mat4 projection_mat, glm::mat4 view_mat,
                                         glm::mat4 model_mat,
                                         const std::vector<float>& vertices,
                                         const std::vector<uint8_t>& rgb_data) {
    shader_->Bind();
    //mesh_->DrawPointCloud(projection_mat, view_mat, model_mat, vertices, rgb_data, shader_id_);
    glUseProgram(0);
    tango_gl::util::CheckGlError("Pointcloud::Render");
  }

}  // namespace rgb_depth_sync
