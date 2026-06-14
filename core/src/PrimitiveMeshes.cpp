#include "ma/PrimitiveMeshes.h"

#include <array>
#include <vector>

namespace ma {

namespace {
// Append an axis-aligned box [c-half, c+half] as 12 flat-shaded triangles.
void appendBox(std::vector<Eigen::Vector3d>& V, std::vector<Eigen::Vector3d>& N,
               std::vector<Eigen::Vector3i>& F, const Eigen::Vector3d& c,
               const Eigen::Vector3d& half) {
  // 8 corners
  std::array<Eigen::Vector3d, 8> p;
  for (int i = 0; i < 8; ++i) {
    p[i] = c + Eigen::Vector3d((i & 1) ? half.x() : -half.x(),
                               (i & 2) ? half.y() : -half.y(),
                               (i & 4) ? half.z() : -half.z());
  }
  // 6 faces as (4 corner indices, outward normal)
  struct Face { int a, b, c, d; Eigen::Vector3d n; };
  const std::array<Face, 6> faces = {{
      {0, 2, 6, 4, {-1, 0, 0}},  // -X
      {1, 5, 7, 3, {1, 0, 0}},   // +X
      {0, 4, 5, 1, {0, -1, 0}},  // -Y
      {2, 3, 7, 6, {0, 1, 0}},   // +Y
      {0, 1, 3, 2, {0, 0, -1}},  // -Z
      {4, 6, 7, 5, {0, 0, 1}},   // +Z
  }};
  for (const auto& f : faces) {
    const int base = static_cast<int>(V.size());
    for (int k : {f.a, f.b, f.c, f.d}) {
      V.push_back(p[k]);
      N.push_back(f.n);
    }
    F.push_back({base + 0, base + 1, base + 2});
    F.push_back({base + 0, base + 2, base + 3});
  }
}

Mesh assemble(const std::vector<Eigen::Vector3d>& V,
              const std::vector<Eigen::Vector3d>& N,
              const std::vector<Eigen::Vector3i>& F) {
  Mesh m;
  m.V.resize(static_cast<Eigen::Index>(V.size()), 3);
  m.VN.resize(static_cast<Eigen::Index>(N.size()), 3);
  m.F.resize(static_cast<Eigen::Index>(F.size()), 3);
  for (size_t i = 0; i < V.size(); ++i) m.V.row(static_cast<Eigen::Index>(i)) = V[i].transpose();
  for (size_t i = 0; i < N.size(); ++i) m.VN.row(static_cast<Eigen::Index>(i)) = N[i].transpose();
  for (size_t i = 0; i < F.size(); ++i) m.F.row(static_cast<Eigen::Index>(i)) = F[i].transpose();
  return m;
}
}  // namespace

Mesh makeBox(double sx, double sy, double sz) {
  std::vector<Eigen::Vector3d> V, N;
  std::vector<Eigen::Vector3i> F;
  appendBox(V, N, F, Eigen::Vector3d::Zero(),
            Eigen::Vector3d(sx * 0.5, sy * 0.5, sz * 0.5));
  Mesh m = assemble(V, N, F);
  m.name = "box";
  return m;
}

Mesh makeCylinder(double radius, double height, int segments) {
  if (segments < 3) segments = 3;
  std::vector<Eigen::Vector3d> V, N;
  std::vector<Eigen::Vector3i> F;
  const double hz = height * 0.5;
  const double tau = 2.0 * 3.14159265358979323846;
  // side wall — shared ring vertices so the wall is one connected patch
  const int bottom = 0;                       // ring start indices
  const int top = segments;
  for (int i = 0; i < segments; ++i) {
    const double a = tau * i / segments;
    const Eigen::Vector3d d(std::cos(a), std::sin(a), 0);
    V.push_back({radius * d.x(), radius * d.y(), -hz}); N.push_back(d);
  }
  for (int i = 0; i < segments; ++i) {
    const double a = tau * i / segments;
    const Eigen::Vector3d d(std::cos(a), std::sin(a), 0);
    V.push_back({radius * d.x(), radius * d.y(), hz}); N.push_back(d);
  }
  for (int i = 0; i < segments; ++i) {
    const int i1 = (i + 1) % segments;
    F.push_back({bottom + i, bottom + i1, top + i1});
    F.push_back({bottom + i, top + i1, top + i});
  }
  // caps (triangle fans)
  for (int cap = 0; cap < 2; ++cap) {
    const double z = cap ? hz : -hz;
    const Eigen::Vector3d nz(0, 0, cap ? 1.0 : -1.0);
    const int center = static_cast<int>(V.size());
    V.push_back({0, 0, z}); N.push_back(nz);
    const int ring = static_cast<int>(V.size());
    for (int i = 0; i < segments; ++i) {
      const double a = tau * i / segments;
      V.push_back({radius * std::cos(a), radius * std::sin(a), z});
      N.push_back(nz);
    }
    for (int i = 0; i < segments; ++i) {
      const int a = ring + i;
      const int b = ring + (i + 1) % segments;
      if (cap) F.push_back({center, a, b});
      else F.push_back({center, b, a});
    }
  }
  Mesh m = assemble(V, N, F);
  m.name = "cylinder";
  return m;
}

Mesh makeDemoPart() {
  std::vector<Eigen::Vector3d> V, N;
  std::vector<Eigen::Vector3i> F;
  // base slab
  appendBox(V, N, F, {0, 0, 0}, {60, 30, 6});
  // two bosses of different heights -> asymmetric, easy to read orientation
  appendBox(V, N, F, {-32, 0, 12}, {12, 18, 12});
  appendBox(V, N, F, {30, 0, 9}, {16, 20, 9});
  Mesh m = assemble(V, N, F);
  m.name = "demo-part";
  return m;
}

}  // namespace ma
