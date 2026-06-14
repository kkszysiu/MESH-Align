#include "ma/Orienter.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace ma {

namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

// Build the part->datum transform from an orthonormal frame (X,Y,Z) and origin.
Eigen::Matrix4d makeTransform(const Eigen::Vector3d& X, const Eigen::Vector3d& Y,
                              const Eigen::Vector3d& Z, const Eigen::Vector3d& origin) {
  Eigen::Matrix3d R;
  R.row(0) = X.transpose();
  R.row(1) = Y.transpose();
  R.row(2) = Z.transpose();
  if (R.determinant() < 0) R.row(1) = -R.row(1);  // keep right-handed
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() = R;
  T.topRightCorner<3, 1>() = -R * origin;
  return T;
}

// Orthonormalise (X candidate) against Z, producing a full right-handed frame.
void buildFrame(const Eigen::Vector3d& Zin, const Eigen::Vector3d& Xhint,
                Eigen::Vector3d& X, Eigen::Vector3d& Y, Eigen::Vector3d& Z) {
  Z = Zin.normalized();
  Eigen::Vector3d x = Xhint - Xhint.dot(Z) * Z;
  if (x.norm() < 1e-6) {
    // pick any axis not parallel to Z
    const Eigen::Vector3d ref =
        (std::abs(Z.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    x = ref - ref.dot(Z) * Z;
  }
  X = x.normalized();
  Y = Z.cross(X).normalized();
  X = Y.cross(Z).normalized();
}

OrientationResult pcaFallback(const Mesh& mesh) {
  const Eigen::Vector3d c = mesh.centroid();
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (Eigen::Index i = 0; i < mesh.V.rows(); ++i) {
    const Eigen::Vector3d d = mesh.V.row(i).transpose() - c;
    C += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);  // ascending
  Eigen::Vector3d X, Y, Z;
  buildFrame(es.eigenvectors().col(0), es.eigenvectors().col(2), X, Y, Z);  // Z=thin axis
  OrientationResult r;
  r.transform = makeTransform(X, Y, Z, c);
  r.eulerXYZ_deg = r.transform.topLeftCorner<3, 3>().eulerAngles(0, 1, 2) * kRad2Deg;
  r.confidence = 0.25;
  r.method = "PCA fallback";
  return r;
}
}  // namespace

OrientationResult orient(const Mesh& mesh, std::vector<DetectedPlane>& planes,
                         const std::vector<DetectedCylinder>& cylinders,
                         const SymmetryPlane& symmetry) {
  if (planes.empty() && cylinders.empty()) return pcaFallback(mesh);

  const Eigen::Vector3d centroid = mesh.centroid();

  // --- choose Top (Z): largest-area plane, else dominant bore axis ---
  int topIdx = -1;
  double maxArea = 0.0;
  for (size_t i = 0; i < planes.size(); ++i)
    if (planes[i].area > maxArea) { maxArea = planes[i].area; topIdx = static_cast<int>(i); }

  Eigen::Vector3d Zhint;
  std::string method;
  if (topIdx >= 0) {
    Zhint = planes[topIdx].normal;
    // orient outward (away from part centre)
    if (Zhint.dot(planes[topIdx].point - centroid) < 0) Zhint = -Zhint;
    method = "Largest face -> Top";
  } else {
    Zhint = cylinders.front().axisDir;
    method = "Bore axis -> Top";
  }

  // --- choose Right (X): symmetry normal, else a secondary plane, else bore ---
  Eigen::Vector3d Xhint = Eigen::Vector3d::UnitX();
  int rightIdx = -1;
  // Symmetry is only useful for Right if its plane is not (nearly) parallel to Top.
  const bool symUsable =
      symmetry.score > 0.6 && std::abs(symmetry.normal.dot(Zhint)) < 0.9;
  if (symUsable) {
    Xhint = symmetry.normal;
    method += "; symmetry -> Right";
  } else {
    double best = 0.0;
    for (size_t i = 0; i < planes.size(); ++i) {
      if (static_cast<int>(i) == topIdx) continue;
      const double perp = 1.0 - std::abs(planes[i].normal.dot(Zhint));  // prefer ⟂ to Z
      const double s = perp * planes[i].confidence;
      if (s > best) { best = s; Xhint = planes[i].normal; rightIdx = static_cast<int>(i); }
    }
    if (rightIdx >= 0) method += "; secondary face -> Right";
    else if (!cylinders.empty()) { Xhint = cylinders.front().axisDir; method += "; bore -> Right"; }
  }

  Eigen::Vector3d X, Y, Z;
  buildFrame(Zhint, Xhint, X, Y, Z);

  OrientationResult r;
  r.transform = makeTransform(X, Y, Z, centroid);
  r.eulerXYZ_deg = r.transform.topLeftCorner<3, 3>().eulerAngles(0, 1, 2) * kRad2Deg;
  r.method = method;
  r.topPlaneId = topIdx;
  r.rightPlaneId = rightIdx;

  // confidence: blend top-plane planarity, symmetry, and (orthogonality of X to Z)
  const double topConf = topIdx >= 0 ? planes[topIdx].confidence : 0.4;
  const double symConf = std::clamp(symmetry.score, 0.0, 1.0);
  r.confidence = std::clamp(0.55 * topConf + 0.30 * symConf + 0.15, 0.0, 1.0);

  // mark roles for the UI
  for (auto& p : planes) p.role = DatumRole::None;
  if (topIdx >= 0) planes[static_cast<size_t>(topIdx)].role = DatumRole::Top;
  if (rightIdx >= 0) planes[static_cast<size_t>(rightIdx)].role = DatumRole::Right;

  return r;
}

}  // namespace ma
