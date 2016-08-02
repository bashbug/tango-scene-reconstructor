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

#ifndef TANGOSCENERECONSTRUCTOR_POINT_CLOUD_DRAWABLE_H_
#define TANGOSCENERECONSTRUCTOR_POINT_CLOUD_DRAWABLE_H_

#include <tango-gl/util.h>
#include "jni.h"

namespace tango_scene_reconstructor {

  class PointCloudDrawable {
    public:
      PointCloudDrawable();
      void DeleteGlResources();
      void Render(glm::mat4 projection_mat,
                  glm::mat4 view_mat,
                  glm::mat4 model_mat,
                  const std::vector<float>& vertices,
                  const std::vector<uint8_t>& rgb_data);

      void Render(glm::mat4 projection_mat,
                  glm::mat4 view_mat,
                  glm::mat4 model_mat,
                  const std::vector<float>& vertices,
                  const std::vector<unsigned int>& indices,
                  const std::vector<uint8_t>& colors);

    private:
      GLuint vertex_buffer_;
      GLuint shader_program_;
      GLuint vertices_handle_;
      GLuint mvp_handle_;
      GLuint color_handle_;
      GLuint color_buffer_;
      GLuint indices_handle_;
      GLuint indices_buffer_;
  };

}  // namespace tango_scene_reconstructor

#endif  // TANGOSCENERECONSTRUCTOR_POINT_CLOUD_DRAWABLE_H_
