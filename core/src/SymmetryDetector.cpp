#include "ma/SymmetryDetector.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <nanoflann.hpp>

namespace ma {

namespace {
using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, 3, nanoflann::metric_L2>;

double meanReflectedDistance(const Mesh& mesh, const KDTree& tree,
                             const Eigen::Vector3d& c, const Eigen::Vector3d& n, int stride) {
  double acc = 0.0;
  int count = 0;
  for (Eigen::Index i = 0; i < mesh.V.rows(); i += stride) {
    const Eigen::Vector3d p = mesh.V.row(i).transpose();
    const Eigen::Vector3d r = p - 2.0 * (p - c).dot(n) * n;  // reflect across plane
    const double query[3] = {r.x(), r.y(), r.z()};
    size_t idx = 0;
    double d2 = 0.0;
    nanoflann::KNNResultSet<double> res(1);
    res.init(&idx, &d2);
    tree.index_->findNeighbors(res, query);
    acc += std::sqrt(d2);
    ++count;
  }
  return count ? acc / count : 0.0;
}
}  // namespace

SymmetryPlane detectSymmetry(const Mesh& mesh, int maxSamples) {
  SymmetryPlane best;
  if (mesh.V.rows() < 4) return best;

  const Eigen::Vector3d centroid = mesh.centroid();
  // Principal axes from vertex covariance.
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (Eigen::Index i = 0; i < mesh.V.rows(); ++i) {
    const Eigen::Vector3d d = mesh.V.row(i).transpose() - centroid;
    C += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);

  KDTree tree(3, std::cref(mesh.V), 16);
  const int stride = std::max<int>(1, static_cast<int>(mesh.V.rows() / std::max(1, maxSamples)));
  const double diag = std::max(1e-9, mesh.diagonal());

  best.point = centroid;
  best.score = -1.0;
  double bestMean = 1e30;
  for (int k = 0; k < 3; ++k) {
    const Eigen::Vector3d n = es.eigenvectors().col(k).normalized();
    const double m = meanReflectedDistance(mesh, tree, centroid, n, stride);
    const double score = std::exp(-m / (0.02 * diag));
    if (score > best.score) {
      best.score = score;
      best.normal = n;
      bestMean = m;
    }
  }
  (void)bestMean;
  return best;
}

}  // namespace ma
