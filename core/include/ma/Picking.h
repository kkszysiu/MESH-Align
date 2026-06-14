#pragma once
#include "ma/Geometry.h"
#include "ma/Mesh.h"

#include <array>
#include <vector>

namespace ma {

struct RayHit {
  bool hit = false;
  int face = -1;
  double t = 0.0;
  Eigen::Vector3d point{0, 0, 0};
};

// Closest ray-mesh intersection (Moller-Trumbore, brute force over faces).
// `dir` need not be normalised; t is in units of |dir|.
RayHit raycast(const Mesh& mesh, const Eigen::Vector3d& origin, const Eigen::Vector3d& dir);

// Grow a smooth surface patch from `seed` across adjacency: a neighbour joins
// while the dihedral angle to the face it expands from stays below the limit.
// Captures a bore wall (smoothly curving) while stopping at sharp edges.
std::vector<int> growSmoothRegion(const Mesh& mesh, const FaceData& fd,
                                  const std::vector<std::array<int, 3>>& adjacency, int seed,
                                  double maxDihedralDeg = 20.0);

}  // namespace ma
