#include "ma/MeshValidation.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace ma {

namespace {
// Minimal union-find for connected-component counting.
struct DSU {
  std::vector<int> p;
  explicit DSU(int n) : p(n) {
    for (int i = 0; i < n; ++i) p[i] = i;
  }
  int find(int x) {
    while (p[x] != x) x = p[x] = p[p[x]];
    return x;
  }
  void join(int a, int b) { p[find(a)] = find(b); }
};

uint64_t edgeKey(int a, int b, int64_t n) {
  if (a > b) std::swap(a, b);
  return static_cast<uint64_t>(a) * static_cast<uint64_t>(n) + static_cast<uint64_t>(b);
}
}  // namespace

ValidationReport validate(const Mesh& mesh) {
  ValidationReport r;
  const int64_t nv = mesh.V.rows();
  const int64_t nf = mesh.F.rows();
  if (nv == 0 || nf == 0) {
    r.notes.push_back("Empty mesh");
    return r;
  }

  // --- duplicate vertices (exact-coincident within a tiny epsilon) ---
  {
    const double eps = std::max(1e-9, 1e-7 * mesh.diagonal());
    const double inv = 1.0 / eps;
    std::unordered_map<uint64_t, int> seen;
    seen.reserve(static_cast<size_t>(nv));
    auto quant = [&](double v) { return static_cast<int64_t>(std::llround(v * inv)); };
    int dups = 0;
    for (Eigen::Index i = 0; i < nv; ++i) {
      uint64_t k = static_cast<uint64_t>(quant(mesh.V(i, 0))) * 73856093u ^
                   static_cast<uint64_t>(quant(mesh.V(i, 1))) * 19349663u ^
                   static_cast<uint64_t>(quant(mesh.V(i, 2))) * 83492791u;
      if (!seen.emplace(k, 1).second) ++dups;
    }
    r.duplicateVertices = dups;
  }

  // --- edges: incidence counts -> boundary / non-manifold ---
  std::unordered_map<uint64_t, int> edgeCount;
  edgeCount.reserve(static_cast<size_t>(nf) * 3);
  DSU dsu(static_cast<int>(nv));
  int degenerate = 0;
  for (Eigen::Index f = 0; f < nf; ++f) {
    const int i0 = mesh.F(f, 0), i1 = mesh.F(f, 1), i2 = mesh.F(f, 2);
    const Eigen::Vector3d a = mesh.V.row(i0);
    const Eigen::Vector3d b = mesh.V.row(i1);
    const Eigen::Vector3d c = mesh.V.row(i2);
    if ((b - a).cross(c - a).norm() < 1e-12) ++degenerate;

    edgeCount[edgeKey(i0, i1, nv)]++;
    edgeCount[edgeKey(i1, i2, nv)]++;
    edgeCount[edgeKey(i2, i0, nv)]++;
    dsu.join(i0, i1);
    dsu.join(i1, i2);
  }
  r.degenerateFaces = degenerate;

  for (const auto& [key, count] : edgeCount) {
    (void)key;
    if (count == 1) ++r.boundaryEdges;
    else if (count > 2) ++r.nonManifoldEdges;
  }

  // --- connected components (over vertices referenced by faces) ---
  std::unordered_map<int, int> roots;
  for (Eigen::Index f = 0; f < nf; ++f)
    for (int k = 0; k < 3; ++k) roots[dsu.find(mesh.F(f, k))] = 1;
  r.components = static_cast<int>(roots.size());

  r.isWatertight = (r.boundaryEdges == 0 && r.nonManifoldEdges == 0);

  if (r.isWatertight) r.notes.push_back("Watertight");
  if (r.boundaryEdges > 0)
    r.notes.push_back(std::to_string(r.boundaryEdges) + " boundary edges (open mesh / holes)");
  if (r.nonManifoldEdges > 0)
    r.notes.push_back(std::to_string(r.nonManifoldEdges) + " non-manifold edges");
  if (r.components > 1)
    r.notes.push_back(std::to_string(r.components) + " disconnected components");
  if (r.degenerateFaces > 0)
    r.notes.push_back(std::to_string(r.degenerateFaces) + " degenerate faces");

  return r;
}

}  // namespace ma
