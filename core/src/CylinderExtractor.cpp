#include "ma/CylinderExtractor.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace ma {

namespace {
// Build an orthonormal basis (e1,e2) spanning the plane perpendicular to `axis`.
void perpBasis(const Eigen::Vector3d& axis, Eigen::Vector3d& e1, Eigen::Vector3d& e2) {
  const Eigen::Vector3d ref =
      (std::abs(axis.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  e1 = axis.cross(ref).normalized();
  e2 = axis.cross(e1).normalized();
}

struct Candidate {
  Eigen::Vector3d axisPoint, axisDir;
  double radius = 0;
  bool valid = false;
};

// Fit a cylinder from two oriented points (centroid + normal).
Candidate fitFromPair(const Eigen::Vector3d& c1, const Eigen::Vector3d& n1,
                      const Eigen::Vector3d& c2, const Eigen::Vector3d& n2) {
  Candidate cand;
  Eigen::Vector3d axis = n1.cross(n2);
  if (axis.norm() < 1e-6) return cand;
  axis.normalize();

  Eigen::Vector3d e1, e2;
  perpBasis(axis, e1, e2);
  Eigen::Vector2d c1p(c1.dot(e1), c1.dot(e2));
  Eigen::Vector2d c2p(c2.dot(e1), c2.dot(e2));
  Eigen::Vector2d n1p(n1.dot(e1), n1.dot(e2));
  Eigen::Vector2d n2p(n2.dot(e1), n2.dot(e2));
  if (n1p.norm() < 1e-6 || n2p.norm() < 1e-6) return cand;
  n1p.normalize();
  n2p.normalize();

  const Eigen::Vector2d dn = n1p - n2p;
  const double denom = dn.squaredNorm();
  if (denom < 1e-9) return cand;
  const double r = (c1p - c2p).dot(dn) / denom;
  const Eigen::Vector2d op = c1p - r * n1p;  // 2D centre

  cand.axisDir = axis;
  cand.axisPoint = op.x() * e1 + op.y() * e2 + c1.dot(axis) * axis;
  cand.radius = std::abs(r);
  cand.valid = cand.radius > 1e-9;
  return cand;
}

double distToAxis(const Eigen::Vector3d& p, const Eigen::Vector3d& ap,
                  const Eigen::Vector3d& ad, Eigen::Vector3d& radialDir) {
  const Eigen::Vector3d w = p - ap;
  const Eigen::Vector3d perp = w - w.dot(ad) * ad;
  const double d = perp.norm();
  radialDir = d > 1e-12 ? (perp / d).eval() : Eigen::Vector3d::UnitX();
  return d;
}
}  // namespace

std::vector<DetectedCylinder> extractCylinders(const Mesh& mesh, const FaceData& fd,
                                               const std::vector<char>& skip,
                                               const CylinderExtractionParams& params) {
  const Eigen::Index nf = mesh.F.rows();
  const double diag = mesh.diagonal();
  const double tol = params.radialToleranceFrac * diag;

  std::vector<int> cand;
  cand.reserve(static_cast<size_t>(nf));
  for (Eigen::Index f = 0; f < nf; ++f) {
    if (!skip.empty() && skip[static_cast<size_t>(f)]) continue;
    if (fd.area(f) > 0) cand.push_back(static_cast<int>(f));
  }

  std::vector<char> used(static_cast<size_t>(nf), 0);
  std::vector<DetectedCylinder> cylinders;
  std::mt19937 rng(12345);

  for (int outer = 0; outer < params.maxCylinders; ++outer) {
    std::vector<int> active;
    for (int f : cand)
      if (!used[static_cast<size_t>(f)]) active.push_back(f);
    if (static_cast<int>(active.size()) < params.minInliers) break;
    std::uniform_int_distribution<int> pick(0, static_cast<int>(active.size()) - 1);

    Candidate best;
    std::vector<int> bestInliers;
    for (int it = 0; it < params.ransacIterations; ++it) {
      const int fa = active[pick(rng)];
      const int fb = active[pick(rng)];
      if (fa == fb) continue;
      Candidate c = fitFromPair(fd.centroid.row(fa).transpose(), fd.normal.row(fa).transpose(),
                                fd.centroid.row(fb).transpose(), fd.normal.row(fb).transpose());
      if (!c.valid || c.radius > diag) continue;

      std::vector<int> inliers;
      for (int f : active) {
        Eigen::Vector3d radial;
        const double d = distToAxis(fd.centroid.row(f).transpose(), c.axisPoint, c.axisDir, radial);
        if (std::abs(d - c.radius) > tol) continue;
        const Eigen::Vector3d nf3 = fd.normal.row(f).transpose();
        if (std::abs(nf3.dot(c.axisDir)) > 0.25) continue;     // normal ⟂ axis
        if (std::abs(nf3.dot(radial)) < 0.8) continue;          // normal ∥ radial
        inliers.push_back(f);
      }
      if (inliers.size() > bestInliers.size()) {
        best = c;
        bestInliers = std::move(inliers);
      }
    }

    const int need = std::max(params.minInliers,
                              static_cast<int>(params.minInlierFrac * active.size()));
    if (static_cast<int>(bestInliers.size()) < need) break;

    // Refine radius as the mean inlier distance; keep axis.
    double rsum = 0.0;
    for (int f : bestInliers) {
      Eigen::Vector3d radial;
      rsum += distToAxis(fd.centroid.row(f).transpose(), best.axisPoint, best.axisDir, radial);
    }
    best.radius = rsum / bestInliers.size();

    // axial extent: span of inlier vertices projected onto the axis
    double amin = std::numeric_limits<double>::infinity();
    double amax = -std::numeric_limits<double>::infinity();
    for (int f : bestInliers) {
      for (int k = 0; k < 3; ++k) {
        const double a = mesh.V.row(mesh.F(f, k)).dot(best.axisDir);
        amin = std::min(amin, a);
        amax = std::max(amax, a);
      }
    }
    const double length = (amax > amin) ? (amax - amin) : 0.0;

    for (int f : bestInliers) used[static_cast<size_t>(f)] = 1;

    DetectedCylinder cyl;
    cyl.axisPoint = best.axisPoint;
    cyl.axisDir = best.axisDir.normalized();
    cyl.radius = best.radius;
    cyl.length = length;
    cyl.inlierFaces = bestInliers;
    cyl.confidence = std::clamp(static_cast<double>(bestInliers.size()) /
                                    std::max(1.0, 0.15 * active.size()),
                                0.0, 1.0);
    cyl.label = "Bore " + std::to_string(cylinders.size());
    cylinders.push_back(std::move(cyl));
  }

  std::sort(cylinders.begin(), cylinders.end(),
            [](const DetectedCylinder& a, const DetectedCylinder& b) {
              return a.inlierFaces.size() > b.inlierFaces.size();
            });
  return cylinders;
}

}  // namespace ma
