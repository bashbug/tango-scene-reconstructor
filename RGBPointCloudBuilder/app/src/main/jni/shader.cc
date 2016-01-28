#include <stdlib.h>
#include "tango-gl/util.h"
#include "rgb-depth-sync/shader.h"

namespace rgb_depth_sync {

  Shader::Shader() { }

  Shader::~Shader() {
    glDetachShader(shader_program_, vertex_shader_);
    glDetachShader(shader_program_, fragment_shader_);
    glDeleteShader(vertex_shader_);
    glDeleteShader(fragment_shader_);
    glDeleteProgram(shader_program_);
  }

  void Shader::Bind() {
    // disable anything related to the depth buffer
    glDisable(GL_DEPTH_TEST);
    tango_gl::util::CheckGlError("Shader glDisable(GL_DEPTH_TEST)");
    // use the given shader program for rendering
    glUseProgram(shader_program_);
    tango_gl::util::CheckGlError("Shader glUseProgram(shader_program_)");
  }

  GLuint Shader::CreateProgram(const char* vertex_source, const char* fragment_source) {

    // create a shader program
    shader_program_ = glCreateProgram();

    // create vertex and fragment shader from the given shader_source
    GLuint vertex_shader_ = CreateShader(GL_VERTEX_SHADER, vertex_source);
    tango_gl::util::CheckGlError("Shader CreateShader(GL_VERTEX_SHADER, vertex_source)");
    if (!vertex_shader_) {
      return 0;
    }

    GLuint fragment_shader_ = CreateShader(GL_FRAGMENT_SHADER, fragment_source);
    tango_gl::util::CheckGlError("Shader CreateShader(GL_FRAGMENT_SHADER, fragment_source)");
    if (!fragment_shader_) {
      return 0;
    }

    if (shader_program_) {
      // attach vertex and fragment shader to shader program
      glAttachShader(shader_program_, vertex_shader_);
      tango_gl::util::CheckGlError("Shader glAttachShader(shader_program_, vertex_shader_)");
      glAttachShader(shader_program_, fragment_shader_);
      tango_gl::util::CheckGlError("Shader glAttachShader(shader_program_, fragment_shader_)");

      // Bind the vertices and texture coordinate attribute locations.
      glBindAttribLocation(shader_program_, 0, "vertex");
      tango_gl::util::CheckGlError("Shader glBindAttribLocation(shader_program_, 0, \"vertex\")");
      glBindAttribLocation(shader_program_, 1, "textureCoords");
      tango_gl::util::CheckGlError("Shader glBindAttribLocation(shader_program_, 1, \"textureCoords\")");

      // link shader program
      glLinkProgram(shader_program_);
      tango_gl::util::CheckGlError("Shader glLinkProgram(shader_program_)");

      glValidateProgram(shader_program_);
      tango_gl::util::CheckGlError("Shader glValidateProgram(shader_program_)");

      // check if linking has failed and print error message to LOGE
      GLint link_status = GL_FALSE;
      glGetProgramiv(shader_program_, GL_LINK_STATUS, &link_status);
      if (link_status != GL_TRUE) {
        GLint buf_length = 0;
        glGetProgramiv(shader_program_, GL_INFO_LOG_LENGTH, &buf_length);
        if (buf_length) {
          char* buf = (char*) malloc(buf_length);
          if (buf) {
            glGetProgramInfoLog(shader_program_, buf_length, NULL, buf);
            LOGE("Could not link program:\n%s\n", buf);
            free(buf);
          }
        }
        glDeleteProgram(shader_program_);
        shader_program_ = 0;
      }
    } else{
      LOGE("Shader not created");
    }

    return shader_program_;
  }

  GLuint Shader::CreateShader(GLenum shader_type, const char *shader_source) {
    // create vertex or fragment shader from source string
    GLuint shader = glCreateShader(shader_type);

    tango_gl::util::CheckGlError("Shader glCreateShader(shader_type)");

    if (shader) {
      // set the shader source string and his size
      glShaderSource(shader, 1, &shader_source, NULL);
      tango_gl::util::CheckGlError("Shader glShaderSource(shader, 1, &shader_source, NULL)");
      // compile shader program after setting its source shaders
      glCompileShader(shader);
      tango_gl::util::CheckGlError("Shader glCompileShader(shader)");

      // check if compiling has failed and print error message to LOGE
      GLint compiled = 0;
      glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
      if (!compiled) {
        GLint info_len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &info_len);
        if (info_len) {
          char* buf = (char*) malloc(info_len);
          if (buf) {
            glGetShaderInfoLog(shader, info_len, NULL, buf);
            LOGE("Could not compile shader %d:\n%s\n", shader_type, buf);
            free(buf);
          }
          glDeleteShader(shader);
          shader = 0;
        }
      }
    }
    return shader;
  }

  void Shader::CheckShaderError(const char* operation) {
    for (GLint error = glGetError(); error; error = glGetError()) {
      LOGE("after %s() glError (0x%x)\n", operation, error);
    }
  }

}  // namespace rgb_depth_sync
