#pragma once
#include "ma_gl/GL.h"

namespace ma::gl {

// Offscreen color+depth target so the 3D scene can be shown inside an ImGui panel.
class Framebuffer {
 public:
  ~Framebuffer();
  void resize(int w, int h);   // (re)allocates if size changed
  void bind() const;           // sets viewport too
  void unbind() const;
  GLuint texture() const { return color_; }
  int width() const { return w_; }
  int height() const { return h_; }

 private:
  void destroy();
  GLuint fbo_ = 0, color_ = 0, depth_ = 0;
  int w_ = 0, h_ = 0;
};

}  // namespace ma::gl
