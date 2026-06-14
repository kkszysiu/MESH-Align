#include "ma/Mesh.h"
#include "ma/Types.h"

namespace ma {

Eigen::Vector3d Mesh::bboxMin() const {
  if (V.rows() == 0) return Eigen::Vector3d::Zero();
  return V.colwise().minCoeff().transpose();
}

Eigen::Vector3d Mesh::bboxMax() const {
  if (V.rows() == 0) return Eigen::Vector3d::Zero();
  return V.colwise().maxCoeff().transpose();
}

Eigen::Vector3d Mesh::center() const { return 0.5 * (bboxMin() + bboxMax()); }

Eigen::Vector3d Mesh::centroid() const {
  if (V.rows() == 0) return Eigen::Vector3d::Zero();
  return V.colwise().mean().transpose();
}

double Mesh::diagonal() const { return (bboxMax() - bboxMin()).norm(); }

double Mesh::surfaceArea() const {
  double area = 0.0;
  for (Eigen::Index f = 0; f < F.rows(); ++f) {
    const Eigen::Vector3d a = V.row(F(f, 0));
    const Eigen::Vector3d b = V.row(F(f, 1));
    const Eigen::Vector3d c = V.row(F(f, 2));
    area += 0.5 * (b - a).cross(c - a).norm();
  }
  return area;
}

double Mesh::volume() const {
  double vol = 0.0;
  for (Eigen::Index f = 0; f < F.rows(); ++f) {
    const Eigen::Vector3d a = V.row(F(f, 0));
    const Eigen::Vector3d b = V.row(F(f, 1));
    const Eigen::Vector3d c = V.row(F(f, 2));
    vol += a.dot(b.cross(c)) / 6.0;
  }
  return vol;
}

const char* stageName(Stage s) {
  switch (s) {
    case Stage::Idle: return "Idle";
    case Stage::Validation: return "Mesh validation";
    case Stage::Normals: return "Normal estimation";
    case Stage::Planes: return "Plane extraction";
    case Stage::Cylinders: return "Cylinder detection";
    case Stage::Symmetry: return "Symmetry detection";
    case Stage::Orientation: return "Auto-orientation";
    case Stage::Deviation: return "Deviation analysis";
    case Stage::Done: return "Done";
  }
  return "?";
}

}  // namespace ma
