#include "ma/Deviation.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

namespace ma {

namespace {
Eigen::Vector3d faceNormal(const Mesh& m, int f) {
  const Eigen::Vector3d a = m.V.row(m.F(f, 0));
  const Eigen::Vector3d b = m.V.row(m.F(f, 1));
  const Eigen::Vector3d c = m.V.row(m.F(f, 2));
  Eigen::Vector3d n = (b - a).cross(c - a);
  const double l = n.norm();
  return l > 1e-20 ? (n / l) : Eigen::Vector3d(0, 0, 1);
}

void pcaFrame(const Eigen::MatrixXd& V, Eigen::Vector3d& centroid, Eigen::Matrix3d& axes) {
  centroid = V.colwise().mean().transpose();
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (Eigen::Index i = 0; i < V.rows(); ++i) {
    const Eigen::Vector3d d = V.row(i).transpose() - centroid;
    C += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);  // ascending
  axes.col(0) = es.eigenvectors().col(2);                // largest first
  axes.col(1) = es.eigenvectors().col(1);
  axes.col(2) = es.eigenvectors().col(0);
}

double meanClosest(const Mesh& scan, const Bvh& bvh, const Eigen::Matrix4d& T, int stride) {
  const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
  const Eigen::Vector3d t = T.topRightCorner<3, 1>();
  double acc = 0;
  int n = 0;
  for (Eigen::Index i = 0; i < scan.V.rows(); i += stride) {
    acc += bvh.closest(R * scan.V.row(i).transpose() + t).distance;
    ++n;
  }
  return n ? acc / n : 0.0;
}
}  // namespace

IcpResult icpToReference(const Mesh& scan, const Mesh& reference, const Bvh& refBvh, int maxIters,
                         bool pcaInit) {
  IcpResult out;
  if (scan.empty() || reference.empty()) return out;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // ---- coarse init ----
  Eigen::Vector3d cs, cr;
  Eigen::Matrix3d As, Ar;
  pcaFrame(scan.V, cs, As);
  pcaFrame(reference.V, cr, Ar);
  const int stride = std::max<int>(1, static_cast<int>(scan.V.rows() / 2000));

  if (pcaInit) {
    // try principal-axis alignments with sign flips, keep the best.
    double best = std::numeric_limits<double>::infinity();
    const int signs[4][2] = {{1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
    for (auto& s : signs) {
      Eigen::Matrix3d Arr = Ar;
      Arr.col(0) *= s[0];
      Arr.col(1) *= s[1];
      Arr.col(2) = Arr.col(0).cross(Arr.col(1));  // keep right-handed
      const Eigen::Matrix3d R = Arr * As.transpose();
      Eigen::Matrix4d cand = Eigen::Matrix4d::Identity();
      cand.topLeftCorner<3, 3>() = R;
      cand.topRightCorner<3, 1>() = cr - R * cs;
      const double m = meanClosest(scan, refBvh, cand, stride);
      if (m < best) { best = m; T = cand; }
    }
  } else {
    T.topRightCorner<3, 1>() = cr - cs;  // centroid only
  }

  // ---- point-to-plane iterations ----
  double prevRms = 1e30;
  for (int it = 0; it < maxIters; ++it) {
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d t = T.topRightCorner<3, 1>();

    // gather correspondences
    std::vector<Eigen::Vector3d> ps, qs, ns;
    std::vector<double> dists;
    ps.reserve(static_cast<size_t>(scan.V.rows()));
    for (Eigen::Index i = 0; i < scan.V.rows(); ++i) {
      const Eigen::Vector3d p = R * scan.V.row(i).transpose() + t;
      const ClosestPoint cp = refBvh.closest(p);
      ps.push_back(p);
      qs.push_back(cp.point);
      ns.push_back(faceNormal(reference, cp.face));
      dists.push_back(cp.distance);
    }
    // robust rejection at 3x median
    std::vector<double> sorted = dists;
    std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
    const double thresh = 3.0 * std::max(sorted[sorted.size() / 2], 1e-9);

    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    double sse = 0;
    int used = 0;
    for (size_t i = 0; i < ps.size(); ++i) {
      if (dists[i] > thresh) continue;
      const Eigen::Vector3d& p = ps[i];
      const Eigen::Vector3d& q = qs[i];
      const Eigen::Vector3d& n = ns[i];
      Eigen::Matrix<double, 6, 1> a;
      a.head<3>() = p.cross(n);
      a.tail<3>() = n;
      const double e = (p - q).dot(n);
      H += a * a.transpose();
      g -= a * e;
      sse += e * e;
      ++used;
    }
    if (used < 6) break;
    const double rms = std::sqrt(sse / used);
    out.rms = rms;
    out.iterations = it + 1;

    const Eigen::Matrix<double, 6, 1> x = H.ldlt().solve(g);
    const Eigen::Vector3d w = x.head<3>();
    const Eigen::Vector3d dt = x.tail<3>();
    Eigen::Matrix4d inc = Eigen::Matrix4d::Identity();
    const double ang = w.norm();
    inc.topLeftCorner<3, 3>() =
        ang > 1e-12 ? Eigen::AngleAxisd(ang, w / ang).toRotationMatrix() : Eigen::Matrix3d::Identity();
    inc.topRightCorner<3, 1>() = dt;
    T = inc * T;

    if (std::abs(prevRms - rms) < 1e-7 * std::max(1.0, reference.diagonal())) break;
    prevRms = rms;
  }
  out.transform = T;
  return out;
}

DeviationField deviationToReference(const Mesh& scan, const Mesh& reference, const Bvh& refBvh,
                                    const Eigen::Matrix4d& scanToRef, const ToleranceBands& bands) {
  DeviationField fld;
  fld.bands = bands;
  fld.signedDistMm.resize(scan.V.rows());
  const Eigen::Matrix3d R = scanToRef.topLeftCorner<3, 3>();
  const Eigen::Vector3d t = scanToRef.topRightCorner<3, 1>();
  double mn = 1e30, mx = -1e30;
  for (Eigen::Index i = 0; i < scan.V.rows(); ++i) {
    const Eigen::Vector3d p = R * scan.V.row(i).transpose() + t;
    const ClosestPoint cp = refBvh.closest(p);
    const Eigen::Vector3d n = faceNormal(reference, cp.face);
    const double s = (p - cp.point).dot(n);
    const double d = (s >= 0 ? 1.0 : -1.0) * cp.distance;
    fld.signedDistMm(i) = d;
    mn = std::min(mn, d);
    mx = std::max(mx, d);
  }
  fld.minMm = scan.V.rows() ? mn : 0;
  fld.maxMm = scan.V.rows() ? mx : 0;
  return fld;
}

PlaneResidual planeResidual(const Mesh& mesh, const Eigen::Vector3d& point,
                            const Eigen::Vector3d& normal, const std::vector<int>& inlierFaces) {
  PlaneResidual r;
  const Eigen::Vector3d n = normal.normalized();
  double sse = 0.0;
  long count = 0;
  for (int f : inlierFaces) {
    if (f < 0 || f >= mesh.F.rows()) continue;
    for (int k = 0; k < 3; ++k) {
      const double d = (mesh.V.row(mesh.F(f, k)).transpose() - point).dot(n);
      sse += d * d;
      r.maxAbs = std::max(r.maxAbs, std::abs(d));
      ++count;
    }
  }
  r.rms = count ? std::sqrt(sse / count) : 0.0;
  return r;
}

DeviationField deviationToPlane(const Mesh& scan, const Eigen::Vector3d& point,
                                const Eigen::Vector3d& normal, const ToleranceBands& bands) {
  DeviationField fld;
  fld.bands = bands;
  fld.signedDistMm.resize(scan.V.rows());
  const Eigen::Vector3d n = normal.normalized();
  double mn = 1e30, mx = -1e30;
  for (Eigen::Index i = 0; i < scan.V.rows(); ++i) {
    const double d = (scan.V.row(i).transpose() - point).dot(n);
    fld.signedDistMm(i) = d;
    mn = std::min(mn, d);
    mx = std::max(mx, d);
  }
  fld.minMm = scan.V.rows() ? mn : 0;
  fld.maxMm = scan.V.rows() ? mx : 0;
  return fld;
}

}  // namespace ma
