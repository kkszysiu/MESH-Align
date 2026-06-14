#include "ma_gl/Heatmap.h"

#include <array>
#include <cmath>

namespace ma::gl {

Eigen::Vector4f bandColor(float v, float inTol, float warn) {
  const float a = std::abs(v);
  if (a <= inTol) return Eigen::Vector4f(0.25f, 0.78f, 0.35f, 1.0f);   // green
  if (a <= warn) return Eigen::Vector4f(0.95f, 0.82f, 0.25f, 1.0f);     // yellow
  return v > 0 ? Eigen::Vector4f(0.88f, 0.23f, 0.24f, 1.0f)             // red  (proud)
               : Eigen::Vector4f(0.23f, 0.45f, 0.90f, 1.0f);            // blue (recessed)
}

GLuint makeBandLutTexture() {
  GLuint tex = 0;
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  return tex;
}

void updateBandLutTexture(GLuint tex, float inTol, float warn, float range) {
  constexpr int N = 256;
  std::array<unsigned char, N * 4> px;
  for (int i = 0; i < N; ++i) {
    const float t = static_cast<float>(i) / (N - 1);
    const float v = -range + 2.0f * range * t;
    const Eigen::Vector4f c = bandColor(v, inTol, warn);
    px[static_cast<size_t>(i) * 4 + 0] = static_cast<unsigned char>(c.x() * 255);
    px[static_cast<size_t>(i) * 4 + 1] = static_cast<unsigned char>(c.y() * 255);
    px[static_cast<size_t>(i) * 4 + 2] = static_cast<unsigned char>(c.z() * 255);
    px[static_cast<size_t>(i) * 4 + 3] = 255;
  }
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, N, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
}

}  // namespace ma::gl
