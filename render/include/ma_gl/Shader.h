#pragma once
#include "ma_gl/GL.h"
#include <Eigen/Core>
#include <string>

namespace ma::gl {

// Minimal program wrapper: compile vertex+fragment source, set uniforms.
class Shader {
 public:
  Shader() = default;
  ~Shader();
  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  bool build(const char* vertexSrc, const char* fragmentSrc);
  void use() const;
  GLuint id() const { return program_; }

  void setMat4(const char* name, const Eigen::Matrix4f& m) const;
  void setMat3(const char* name, const Eigen::Matrix3f& m) const;
  void setVec3(const char* name, const Eigen::Vector3f& v) const;
  void setFloat(const char* name, float v) const;
  void setInt(const char* name, int v) const;

 private:
  GLuint program_ = 0;
};

}  // namespace ma::gl
