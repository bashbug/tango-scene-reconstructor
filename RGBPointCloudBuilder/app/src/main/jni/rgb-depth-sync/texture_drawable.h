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

#ifndef RGB_DEPTH_SYNC_TEXTURE_DRAWABLE_H_
#define RGB_DEPTH_SYNC_TEXTURE_DRAWABLE_H_

#include <tango-gl/util.h>
#include "rgb-depth-sync/shader.h"
#include "rgb-depth-sync/texture.h"
#include "rgb-depth-sync/mesh.h"

namespace rgb_depth_sync {

  class TextureDrawable {
    public:
      TextureDrawable(const char* vertex_source,
                      const char* fragment_source);
      ~TextureDrawable();
      void RenderImage(int width, int height);
      void RenderImage(int width, int height, GLuint texture_id);
      void RenderPointCloud(glm::mat4 projection_mat, glm::mat4 view_mat,
                            glm::mat4 model_mat,
                            const std::vector<float>& vertices,
                            const std::vector<uint8_t>& rgb_data);
      GLuint GetTextureId() const { return texture_id_; }
      void SetTextureId(GLuint texture_id) { texture_id_ = texture_id; }
      void SetGrayTexture(void* data, int width, int height);
      void SetRGBTexture(void* data, int width, int height);

    private:
      GLuint texture_id_;
      GLuint shader_id_;
      Shader* shader_;
      Texture* texture_;
      Mesh* mesh_;
  };

} // namespace rgb_depth_sync

#endif // RGB_DEPTH_SYNC_TEXTURE_DRAWABLE_H_
