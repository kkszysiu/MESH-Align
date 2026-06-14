#pragma once
#include "ma_gl/GL.h"
#include "ma_gl/Shader.h"
#include "ma/Mesh.h"
#include <Eigen/Core>

namespace ma::gl {

// Uploads a ma::Mesh and draws it with a simple clay/matcap-style lit shader.
class MeshRenderer {
 public:
  bool init();                       // builds the shaders (needs a current GL context)
  void upload(const ma::Mesh& mesh); // (re)uploads geometry; computes normals if absent
  void uploadScalars(const Eigen::VectorXf& s);  // per-vertex scalar for heatmap mode
  bool hasScalars() const { return hasScalars_; }

  void draw(const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj,
            const Eigen::Matrix4f& model, const Eigen::Vector3f& color);
  // Heatmap: colour by the uploaded scalar via a 1D LUT texture (range [vmin,vmax]).
  void drawHeatmap(const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj,
                   const Eigen::Matrix4f& model, float vmin, float vmax, GLuint lutTex);
  ~MeshRenderer();

 private:
  Shader shader_;
  Shader heatmap_;
  GLuint vao_ = 0, vbo_ = 0, ebo_ = 0, scalarVbo_ = 0;
  GLsizei indexCount_ = 0;
  bool hasScalars_ = false;
};

}  // namespace ma::gl
