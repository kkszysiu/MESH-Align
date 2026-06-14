#pragma once
#include "ma/Mesh.h"

#include <array>
#include <vector>

namespace ma {

// Per-face quantities used across the analysis stages.
struct FaceData {
  Eigen::MatrixXd normal;    // #F x 3 (unit)
  Eigen::VectorXd area;      // #F
  Eigen::MatrixXd centroid;  // #F x 3
  double totalArea = 0.0;
};

FaceData computeFaceData(const Mesh& mesh);

// For each face, the up-to-3 edge-adjacent face indices (-1 where boundary /
// non-manifold). Edges are matched by sorted vertex-index pairs.
std::vector<std::array<int, 3>> triangleAdjacency(const Mesh& mesh);

// Area-weighted PCA of a set of 3D points (rows of P, weights w).
// Returns mean in `centroid`, the eigenvector of the smallest eigenvalue in
// `normal`, and the three ascending eigenvalues in `eigenvalues`.
void weightedPCA(const Eigen::MatrixXd& P, const Eigen::VectorXd& w,
                 Eigen::Vector3d& centroid, Eigen::Vector3d& normal,
                 Eigen::Vector3d& eigenvalues);

}  // namespace ma
