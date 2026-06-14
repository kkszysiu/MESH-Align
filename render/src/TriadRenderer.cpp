#include "ma_gl/TriadRenderer.h"

namespace ma::gl {

namespace {
const char* kVert = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uViewProj;
uniform float uLen;
out vec3 vColor;
void main(){
  vColor = aColor;
  gl_Position = uViewProj * vec4(aPos * uLen, 1.0);
})";

const char* kFrag = R"(#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main(){ FragColor = vec4(vColor, 1.0); })";
}  // namespace

bool TriadRenderer::init() {
  if (!shader_.build(kVert, kFrag)) return false;
  // 3 axes, 2 verts each: position(3) + color(3)
  const float v[] = {
      0, 0, 0, 1, 0, 0,  1, 0, 0, 1, 0, 0,  // X red
      0, 0, 0, 0, 1, 0,  0, 1, 0, 0, 1, 0,  // Y green
      0, 0, 0, 0, 0, 1,  0, 0, 1, 0, 0, 1,  // Z blue
  };
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(v), v, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
  glBindVertexArray(0);
  return true;
}

TriadRenderer::~TriadRenderer() {
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
}

void TriadRenderer::draw(const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj,
                         float length) {
  shader_.use();
  shader_.setMat4("uViewProj", Eigen::Matrix4f(proj * view));
  shader_.setFloat("uLen", length);
  glBindVertexArray(vao_);
  glDrawArrays(GL_LINES, 0, 6);
  glBindVertexArray(0);
}

}  // namespace ma::gl
