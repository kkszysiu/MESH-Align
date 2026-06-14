#include "ma_gl/MeshRenderer.h"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <vector>

namespace ma::gl {

namespace {
const char* kVert = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
uniform mat3 uNormalMat;
out vec3 vN;
out vec3 vP;
void main(){
  vec4 posVS = uView * uModel * vec4(aPos, 1.0);
  vP = posVS.xyz;
  vN = normalize(uNormalMat * aNormal);
  gl_Position = uProj * posVS;
})";

const char* kFrag = R"(#version 330 core
in vec3 vN;
in vec3 vP;
out vec4 FragColor;
uniform vec3 uColor;
void main(){
  vec3 N = normalize(vN);
  if(!gl_FrontFacing) N = -N;
  vec3 V = normalize(-vP);
  vec3 L1 = normalize(vec3(0.5, 0.7, 0.8));
  vec3 L2 = normalize(vec3(-0.6, -0.3, 0.4));
  float diff = max(dot(N, L1), 0.0) * 0.8 + max(dot(N, L2), 0.0) * 0.25;
  float amb = 0.32;
  float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.22;
  vec3 col = uColor * (amb + diff) + vec3(rim);
  FragColor = vec4(col, 1.0);
})";

const char* kHeatVert = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in float aScalar;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
uniform mat3 uNormalMat;
out vec3 vN;
out float vS;
void main(){
  vN = normalize(uNormalMat * aNormal);
  vS = aScalar;
  gl_Position = uProj * uView * uModel * vec4(aPos, 1.0);
})";

const char* kHeatFrag = R"(#version 330 core
in vec3 vN;
in float vS;
out vec4 FragColor;
uniform sampler2D uLut;
uniform float uMin;
uniform float uMax;
void main(){
  float t = clamp((vS - uMin) / max(uMax - uMin, 1e-6), 0.0, 1.0);
  vec3 base = texture(uLut, vec2(t, 0.5)).rgb;
  vec3 N = normalize(vN);
  if(!gl_FrontFacing) N = -N;
  float shade = 0.55 + 0.45 * max(dot(N, normalize(vec3(0.4,0.6,0.8))), 0.0);
  FragColor = vec4(base * shade, 1.0);
})";

// Smooth per-vertex normals from faces (used when the mesh has none).
Eigen::MatrixXd computeVertexNormals(const ma::Mesh& m) {
  Eigen::MatrixXd N = Eigen::MatrixXd::Zero(m.V.rows(), 3);
  for (Eigen::Index f = 0; f < m.F.rows(); ++f) {
    const Eigen::Vector3d a = m.V.row(m.F(f, 0));
    const Eigen::Vector3d b = m.V.row(m.F(f, 1));
    const Eigen::Vector3d c = m.V.row(m.F(f, 2));
    Eigen::Vector3d fn = (b - a).cross(c - a);  // area-weighted
    for (int k = 0; k < 3; ++k) N.row(m.F(f, k)) += fn.transpose();
  }
  for (Eigen::Index i = 0; i < N.rows(); ++i) {
    double n = N.row(i).norm();
    if (n > 1e-12) N.row(i) /= n;
  }
  return N;
}
}  // namespace

bool MeshRenderer::init() {
  return shader_.build(kVert, kFrag) && heatmap_.build(kHeatVert, kHeatFrag);
}

MeshRenderer::~MeshRenderer() {
  if (scalarVbo_) glDeleteBuffers(1, &scalarVbo_);
  if (ebo_) glDeleteBuffers(1, &ebo_);
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
}

void MeshRenderer::uploadScalars(const Eigen::VectorXf& s) {
  if (!vao_ || s.size() == 0) {
    hasScalars_ = false;
    return;
  }
  if (!scalarVbo_) glGenBuffers(1, &scalarVbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, scalarVbo_);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(s.size() * sizeof(float)), s.data(),
               GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void*)0);
  glBindVertexArray(0);
  hasScalars_ = true;
}

void MeshRenderer::drawHeatmap(const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj,
                               const Eigen::Matrix4f& model, float vmin, float vmax, GLuint lutTex) {
  if (!indexCount_ || !hasScalars_) return;
  heatmap_.use();
  heatmap_.setMat4("uModel", model);
  heatmap_.setMat4("uView", view);
  heatmap_.setMat4("uProj", proj);
  Eigen::Matrix3f nm = (view * model).topLeftCorner<3, 3>().inverse().transpose();
  heatmap_.setMat3("uNormalMat", nm);
  heatmap_.setFloat("uMin", vmin);
  heatmap_.setFloat("uMax", vmax);
  heatmap_.setInt("uLut", 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, lutTex);
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void MeshRenderer::upload(const ma::Mesh& mesh) {
  if (mesh.empty()) {
    indexCount_ = 0;
    return;
  }
  const Eigen::MatrixXd& VN =
      (mesh.VN.rows() == mesh.V.rows()) ? mesh.VN : computeVertexNormals(mesh);

  std::vector<float> interleaved;
  interleaved.reserve(static_cast<size_t>(mesh.V.rows()) * 6);
  for (Eigen::Index i = 0; i < mesh.V.rows(); ++i) {
    interleaved.push_back(static_cast<float>(mesh.V(i, 0)));
    interleaved.push_back(static_cast<float>(mesh.V(i, 1)));
    interleaved.push_back(static_cast<float>(mesh.V(i, 2)));
    interleaved.push_back(static_cast<float>(VN(i, 0)));
    interleaved.push_back(static_cast<float>(VN(i, 1)));
    interleaved.push_back(static_cast<float>(VN(i, 2)));
  }
  std::vector<unsigned int> indices;
  indices.reserve(static_cast<size_t>(mesh.F.rows()) * 3);
  for (Eigen::Index f = 0; f < mesh.F.rows(); ++f)
    for (int k = 0; k < 3; ++k)
      indices.push_back(static_cast<unsigned int>(mesh.F(f, k)));
  indexCount_ = static_cast<GLsizei>(indices.size());

  if (!vao_) glGenVertexArrays(1, &vao_);
  if (!vbo_) glGenBuffers(1, &vbo_);
  if (!ebo_) glGenBuffers(1, &ebo_);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)),
               interleaved.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
               indices.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        (void*)(3 * sizeof(float)));
  glBindVertexArray(0);
}

void MeshRenderer::draw(const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj,
                        const Eigen::Matrix4f& model, const Eigen::Vector3f& color) {
  if (!indexCount_) return;
  shader_.use();
  shader_.setMat4("uModel", model);
  shader_.setMat4("uView", view);
  shader_.setMat4("uProj", proj);
  Eigen::Matrix3f nm = (view * model).topLeftCorner<3, 3>().inverse().transpose();
  shader_.setMat3("uNormalMat", nm);
  shader_.setVec3("uColor", color);
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

}  // namespace ma::gl
