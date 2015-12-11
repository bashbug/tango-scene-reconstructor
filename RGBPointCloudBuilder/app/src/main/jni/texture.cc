#include "rgb-depth-sync/texture.h"

namespace rgb_depth_sync {

  Texture::Texture() {
    glGenTextures(1, &texture_id_);
    tango_gl::util::CheckGlError("Texture glGenTextures(1, &texture_id_)");
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    tango_gl::util::CheckGlError("Texture glBindTexture(GL_TEXTURE_2D, texture_id_)");
  }

  void Texture::SetTextureRGBData(void* data, int width, int height) {
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    tango_gl::util::CheckGlError("Texture glBindTexture(GL_TEXTURE_2D, texture_id_)");
    glTexImage2D(GL_TEXTURE_2D, 0,  // mip-map level
                 GL_RGB, width, height, 0,  // border
                 GL_RGB, GL_UNSIGNED_BYTE, data);
    tango_gl::util::CheckGlError("Texture glTexImage2D(...)");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    tango_gl::util::CheckGlError("Texture glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    tango_gl::util::CheckGlError("Texture glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)");
  }

  void Texture::SetTextureGrayData(void* data, int width, int height) {
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    tango_gl::util::CheckGlError("Texture glBindTexture(GL_TEXTURE_2D, texture_id_)");
    glTexImage2D(GL_TEXTURE_2D, 0,  // mip-map level
                 GL_LUMINANCE, width, height, 0,  // border
                 GL_LUMINANCE, GL_UNSIGNED_BYTE, data);
    tango_gl::util::CheckGlError("Texture glTexImage2D(...)");
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    tango_gl::util::CheckGlError("Texture glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    tango_gl::util::CheckGlError("Texture glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)");
  }


  Texture::~Texture(){
    glDeleteTextures(1, &texture_id_);
  }

  void Texture::Bind(GLuint shader_id) {
    // bind texture to GL_TEXTURE2D
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    tango_gl::util::CheckGlError("Texture glBindTexture(GL_TEXTURE_2D, texture_id_)");
    // set GL_TEXTURE1 unit active
    glActiveTexture(GL_TEXTURE1);
    tango_gl::util::CheckGlError("Texture glActiveTexture(GL_TEXTURE1)");
    // texture has uniform location in shader program and has to get the active texture unit id 1
    glUniform1i(glGetUniformLocation(shader_id, "texture"), 1);
    tango_gl::util::CheckGlError("Texture glUniform1i(glGetUniformLocation(shader_id, \"texture\"), 1)");
  }

  void Texture::Bind(GLuint shader_id, GLuint texture_id) {
    // texture has uniform location in shader program and has to get the active texture unit id 0
    glUniform1i(glGetUniformLocation(shader_id, "texture"), 0);
    tango_gl::util::CheckGlError("Texture glUniform1i(glGetUniformLocation(shader_id, \"texture\"), 0)");
    // set GL_TEXTURE0 unit active
    glActiveTexture(GL_TEXTURE0);
    tango_gl::util::CheckGlError("Texture glActiveTexture(GL_TEXTURE0)");
    // bind texture to GL_TEXTURE_EXTERNAL_OES
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id);
    tango_gl::util::CheckGlError("Texture glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id_)");
  }

}