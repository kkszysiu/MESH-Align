#pragma once
#include <Eigen/Core>
#include <string>

namespace ma {

// A triangle mesh in libigl convention:
//   V : #V x 3 vertex positions (double, millimetres)
//   F : #F x 3 triangle vertex indices
// Normals are optional caches; renderers may recompute them.
struct Mesh {
  Eigen::MatrixXd V;     // #V x 3
  Eigen::MatrixXi F;     // #F x 3
  Eigen::MatrixXd VN;    // #V x 3 vertex normals (optional)
  Eigen::MatrixXd FN;    // #F x 3 face normals   (optional)
  std::string units = "mm";
  std::string name;

  bool empty() const { return V.rows() == 0 || F.rows() == 0; }
  Eigen::Index numVertices() const { return V.rows(); }
  Eigen::Index numFaces() const { return F.rows(); }

  Eigen::Vector3d bboxMin() const;
  Eigen::Vector3d bboxMax() const;
  Eigen::Vector3d center() const;     // bbox center
  Eigen::Vector3d centroid() const;   // mean of vertices
  double diagonal() const;            // bbox diagonal length

  double surfaceArea() const;
  // Signed volume via the divergence theorem (meaningful only if watertight).
  double volume() const;
};

}  // namespace ma
