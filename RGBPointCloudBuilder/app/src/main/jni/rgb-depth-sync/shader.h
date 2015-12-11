/*
 * Class for shader program. Provides vertex and fragment shader for color
 * and range image.
 */

#ifndef RGB_DEPTH_SYNC_SHADER_H_
#define RGB_DEPTH_SYNC_SHADER_H_

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

namespace rgb_depth_sync {
  namespace shader {
  // Vertex shader for rendering a color camera texture on full screen.
  static const char kColorCameraVertex[] =
    "precision highp float;\n"
    "precision highp int;\n"
    "attribute vec4 vertex;\n"
    "attribute vec2 textureCoords;\n"
    "varying vec2 f_textureCoords;\n"
    "void main() {\n"
    "  f_textureCoords = textureCoords;\n"
    "  gl_Position = vertex;\n"
    "}\n";
  // Fragment shader for rendering a color texture on full screen.
  static const char kColorCameraFragment[] =
    "#extension GL_OES_EGL_image_external : require\n"
    "precision highp float;\n"
    "precision highp int;\n"
    "uniform samplerExternalOES texture;\n"
    "varying vec2 f_textureCoords;\n"
    "void main() {\n"
    "  vec4 color = texture2D(texture, f_textureCoords);\n"
    "  gl_FragColor = color;;\n"
    "}\n";
  // Vertex shader for rendering a range image texture on full screen.
  static const char kRangeImageVertex[] =
    "precision highp float;\n"
    "precision highp int;\n"
    "attribute vec4 vertex;\n"
    "attribute vec2 textureCoords;\n"
    "varying vec2 f_textureCoords;\n"
    "void main() {\n"
    "  f_textureCoords = textureCoords;\n"
    "  gl_Position = vertex;\n"
    "}\n";
  // Fragment shader for rendering a range image on full screen.
  static const char kRangeImageFragment[] =
    "precision highp float;\n"
    "precision highp int;\n"
    "varying vec2 f_textureCoords;\n"
    "uniform sampler2D texture; \n"
    "void main() {\n"
    "  vec4 color = texture2D(texture, f_textureCoords);\n"
    "  gl_FragColor = color;\n"
    "}\n";

  static const char kPointCloudVertex[] =
    "precision highp float;\n"
    "precision highp int;\n"
    "attribute vec4 vertex;\n"
    "attribute vec3 color;\n"
    "uniform mat4 mvp;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  gl_PointSize = 5.0;\n"
    "  gl_Position = mvp*vertex;\n"
    "  v_color = color;\n"
    "}\n";

  static const char kPointCloudFragment[] =
    "precision highp float;\n"
    "precision highp int;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  gl_FragColor = vec4(vec3(v_color), 1.0);\n"
    "}\n";

  static const glm::mat4 kOpengGL_T_Depth =
    glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
              -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  }  // namespace shader

  class Shader {
    public:
      Shader();
      ~Shader();
      GLuint CreateProgram(const char* vertex_source, const char* fragment_source);
      void Bind();
      void CheckShaderError(const char* operation);
    private:
      GLuint CreateShader(GLenum shader_type, const char *shader_source);
      GLuint shader_program_, vertex_shader_, fragment_shader_;

  };

}  // namespace rgb_depth_sync

#endif  // RGB_DEPTH_SYNC_SHADER_H_
