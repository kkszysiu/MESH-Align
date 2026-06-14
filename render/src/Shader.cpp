#include "ma_gl/Shader.h"

#include <cstdio>
#include <vector>

namespace ma::gl {

namespace {
GLuint compile(GLenum type, const char* src) {
  GLuint sh = glCreateShader(type);
  glShaderSource(sh, 1, &src, nullptr);
  glCompileShader(sh);
  GLint ok = 0;
  glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    GLint len = 0;
    glGetShaderiv(sh, GL_INFO_LOG_LENGTH, &len);
    std::vector<char> log(static_cast<size_t>(len > 1 ? len : 1));
    glGetShaderInfoLog(sh, len, nullptr, log.data());
    std::fprintf(stderr, "[shader] %s compile error:\n%s\n",
                 type == GL_VERTEX_SHADER ? "vertex" : "fragment", log.data());
    glDeleteShader(sh);
    return 0;
  }
  return sh;
}
}  // namespace

Shader::~Shader() {
  if (program_) glDeleteProgram(program_);
}

bool Shader::build(const char* vertexSrc, const char* fragmentSrc) {
  GLuint vs = compile(GL_VERTEX_SHADER, vertexSrc);
  GLuint fs = compile(GL_FRAGMENT_SHADER, fragmentSrc);
  if (!vs || !fs) {
    if (vs) glDeleteShader(vs);
    if (fs) glDeleteShader(fs);
    return false;
  }
  program_ = glCreateProgram();
  glAttachShader(program_, vs);
  glAttachShader(program_, fs);
  glLinkProgram(program_);
  glDeleteShader(vs);
  glDeleteShader(fs);

  GLint ok = 0;
  glGetProgramiv(program_, GL_LINK_STATUS, &ok);
  if (!ok) {
    GLint len = 0;
    glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &len);
    std::vector<char> log(static_cast<size_t>(len > 1 ? len : 1));
    glGetProgramInfoLog(program_, len, nullptr, log.data());
    std::fprintf(stderr, "[shader] link error:\n%s\n", log.data());
    glDeleteProgram(program_);
    program_ = 0;
    return false;
  }
  return true;
}

void Shader::use() const { glUseProgram(program_); }

void Shader::setMat4(const char* name, const Eigen::Matrix4f& m) const {
  glUniformMatrix4fv(glGetUniformLocation(program_, name), 1, GL_FALSE, m.data());
}
void Shader::setMat3(const char* name, const Eigen::Matrix3f& m) const {
  glUniformMatrix3fv(glGetUniformLocation(program_, name), 1, GL_FALSE, m.data());
}
void Shader::setVec3(const char* name, const Eigen::Vector3f& v) const {
  glUniform3fv(glGetUniformLocation(program_, name), 1, v.data());
}
void Shader::setFloat(const char* name, float v) const {
  glUniform1f(glGetUniformLocation(program_, name), v);
}
void Shader::setInt(const char* name, int v) const {
  glUniform1i(glGetUniformLocation(program_, name), v);
}

}  // namespace ma::gl
