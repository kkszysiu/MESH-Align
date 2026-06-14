#include "ma_gl/PlaneRenderer.h"

namespace ma::gl {

namespace {
const char* kVert = R"(#version 330 core
layout(location=0) in vec2 aUV;
uniform mat4 uViewProj;
uniform vec3 uCenter;
uniform vec3 uU;
uniform vec3 uV;
uniform float uHalf;
void main(){
  vec3 p = uCenter + aUV.x * uHalf * uU + aUV.y * uHalf * uV;
  gl_Position = uViewProj * vec4(p, 1.0);
})";

const char* kFrag = R"(#version 330 core
out vec4 FragColor;
uniform vec4 uColor;
void main(){ FragColor = uColor; })";
}  // namespace

bool PlaneRenderer::init() {
  if (!shader_.build(kVert, kFrag)) return false;
  const float quad[] = {-1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1};
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glBindVertexArray(0);
  return true;
}

PlaneRenderer::~PlaneRenderer() {
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
}

void PlaneRenderer::draw(const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj,
                         const Eigen::Vector3f& center, const Eigen::Vector3f& u,
                         const Eigen::Vector3f& v, float halfSize,
                         const Eigen::Vector4f& rgba) {
  shader_.use();
  shader_.setMat4("uViewProj", Eigen::Matrix4f(proj * view));
  shader_.setVec3("uCenter", center);
  shader_.setVec3("uU", u);
  shader_.setVec3("uV", v);
  shader_.setFloat("uHalf", halfSize);
  glUniform4fv(glGetUniformLocation(shader_.id(), "uColor"), 1, rgba.data());

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE);  // translucent: test depth but don't write
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, 6);
  glBindVertexArray(0);
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);
}

}  // namespace ma::gl
