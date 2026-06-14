#include "ma/Geometry.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <unordered_map>

namespace ma {

FaceData computeFaceData(const Mesh& mesh) {
  const Eigen::Index nf = mesh.F.rows();
  FaceData fd;
  fd.normal.resize(nf, 3);
  fd.area.resize(nf);
  fd.centroid.resize(nf, 3);
  fd.totalArea = 0.0;
  for (Eigen::Index f = 0; f < nf; ++f) {
    const Eigen::Vector3d a = mesh.V.row(mesh.F(f, 0));
    const Eigen::Vector3d b = mesh.V.row(mesh.F(f, 1));
    const Eigen::Vector3d c = mesh.V.row(mesh.F(f, 2));
    Eigen::Vector3d n = (b - a).cross(c - a);
    const double twiceArea = n.norm();
    fd.area(f) = 0.5 * twiceArea;
    fd.totalArea += fd.area(f);
    fd.normal.row(f) = (twiceArea > 1e-20 ? (n / twiceArea) : Eigen::Vector3d(0, 0, 1)).transpose();
    fd.centroid.row(f) = ((a + b + c) / 3.0).transpose();
  }
  return fd;
}

std::vector<std::array<int, 3>> triangleAdjacency(const Mesh& mesh) {
  const Eigen::Index nf = mesh.F.rows();
  const int64_t nv = mesh.V.rows();
  // edge key -> first face that owns it (and which slot)
  std::unordered_map<uint64_t, int> owner;
  owner.reserve(static_cast<size_t>(nf) * 3);
  std::vector<std::array<int, 3>> adj(static_cast<size_t>(nf), {-1, -1, -1});

  auto key = [nv](int a, int b) {
    if (a > b) std::swap(a, b);
    return static_cast<uint64_t>(a) * static_cast<uint64_t>(nv) + static_cast<uint64_t>(b);
  };
  for (Eigen::Index f = 0; f < nf; ++f) {
    for (int e = 0; e < 3; ++e) {
      const int v0 = mesh.F(f, e);
      const int v1 = mesh.F(f, (e + 1) % 3);
      const uint64_t k = key(v0, v1);
      auto it = owner.find(k);
      if (it == owner.end()) {
        owner.emplace(k, static_cast<int>(f) * 3 + e);
      } else {
        const int other = it->second / 3;
        const int otherSlot = it->second % 3;
        adj[static_cast<size_t>(f)][static_cast<size_t>(e)] = other;
        adj[static_cast<size_t>(other)][static_cast<size_t>(otherSlot)] = static_cast<int>(f);
      }
    }
  }
  return adj;
}

void weightedPCA(const Eigen::MatrixXd& P, const Eigen::VectorXd& w,
                 Eigen::Vector3d& centroid, Eigen::Vector3d& normal,
                 Eigen::Vector3d& eigenvalues) {
  const double wsum = w.sum();
  centroid = (P.transpose() * w) / std::max(wsum, 1e-20);
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (Eigen::Index i = 0; i < P.rows(); ++i) {
    const Eigen::Vector3d d = P.row(i).transpose() - centroid;
    C += w(i) * d * d.transpose();
  }
  C /= std::max(wsum, 1e-20);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
  eigenvalues = es.eigenvalues();              // ascending
  normal = es.eigenvectors().col(0);            // smallest eigenvalue
}

}  // namespace ma
