#pragma once
#include "ma_gl/GL.h"
#include "ma_gl/Shader.h"
#include <Eigen/Core>

namespace ma::gl {

// Draws translucent datum-plane quads (and could be reused for any quad).
class PlaneRenderer {
 public:
  bool init();
  // Draw a quad centred at `center`, spanning ±halfSize along (u,v), with RGBA.
  void draw(const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj,
            const Eigen::Vector3f& center, const Eigen::Vector3f& u,
            const Eigen::Vector3f& v, float halfSize, const Eigen::Vector4f& rgba);
  ~PlaneRenderer();

 private:
  Shader shader_;
  GLuint vao_ = 0, vbo_ = 0;
};

}  // namespace ma::gl
