#include "ma/Picking.h"

#include <Eigen/Geometry>
#include <cmath>
#include <queue>

namespace ma {

RayHit raycast(const Mesh& mesh, const Eigen::Vector3d& origin, const Eigen::Vector3d& dirIn) {
  RayHit best;
  const Eigen::Vector3d d = dirIn.normalized();
  double bestT = std::numeric_limits<double>::infinity();
  const double eps = 1e-9;
  for (Eigen::Index f = 0; f < mesh.F.rows(); ++f) {
    const Eigen::Vector3d v0 = mesh.V.row(mesh.F(f, 0));
    const Eigen::Vector3d v1 = mesh.V.row(mesh.F(f, 1));
    const Eigen::Vector3d v2 = mesh.V.row(mesh.F(f, 2));
    const Eigen::Vector3d e1 = v1 - v0;
    const Eigen::Vector3d e2 = v2 - v0;
    const Eigen::Vector3d p = d.cross(e2);
    const double det = e1.dot(p);
    if (std::abs(det) < eps) continue;
    const double inv = 1.0 / det;
    const Eigen::Vector3d tvec = origin - v0;
    const double u = tvec.dot(p) * inv;
    if (u < 0.0 || u > 1.0) continue;
    const Eigen::Vector3d q = tvec.cross(e1);
    const double v = d.dot(q) * inv;
    if (v < 0.0 || u + v > 1.0) continue;
    const double t = e2.dot(q) * inv;
    if (t > eps && t < bestT) {
      bestT = t;
      best.hit = true;
      best.face = static_cast<int>(f);
    }
  }
  if (best.hit) {
    best.t = bestT;
    best.point = origin + bestT * d;
  }
  return best;
}

std::vector<int> growSmoothRegion(const Mesh& mesh, const FaceData& fd,
                                  const std::vector<std::array<int, 3>>& adjacency, int seed,
                                  double maxDihedralDeg) {
  (void)mesh;
  const double cosTol = std::cos(maxDihedralDeg * 3.14159265358979323846 / 180.0);
  std::vector<int> region;
  if (seed < 0 || seed >= static_cast<int>(adjacency.size())) return region;
  std::vector<char> seen(adjacency.size(), 0);
  std::queue<int> q;
  q.push(seed);
  seen[static_cast<size_t>(seed)] = 1;
  while (!q.empty()) {
    const int f = q.front();
    q.pop();
    region.push_back(f);
    const Eigen::Vector3d nf = fd.normal.row(f).transpose();
    for (int nb : adjacency[static_cast<size_t>(f)]) {
      if (nb < 0 || seen[static_cast<size_t>(nb)]) continue;
      const Eigen::Vector3d nn = fd.normal.row(nb).transpose();
      if (nf.dot(nn) < cosTol) continue;  // sharp edge -> stop
      seen[static_cast<size_t>(nb)] = 1;
      q.push(nb);
    }
  }
  return region;
}

}  // namespace ma
