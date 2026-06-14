#pragma once
#include "ma_gl/GL.h"
#include "ma_gl/Shader.h"
#include <Eigen/Core>

namespace ma::gl {

// Draws the origin coordinate triad (X=red, Y=green, Z=blue) as three lines.
class TriadRenderer {
 public:
  bool init();
  void draw(const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj, float length);
  ~TriadRenderer();

 private:
  Shader shader_;
  GLuint vao_ = 0, vbo_ = 0;
};

}  // namespace ma::gl
