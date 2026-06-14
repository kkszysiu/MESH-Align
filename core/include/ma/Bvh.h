#pragma once
#include "ma/Mesh.h"

#include <Eigen/Geometry>
#include <vector>

namespace ma {

struct ClosestPoint {
  Eigen::Vector3d point{0, 0, 0};
  int face = -1;
  double distance = 0.0;
};

// Axis-aligned bounding-volume hierarchy over a mesh's triangles for fast
// closest-point queries (used by deviation analysis and ICP).
class Bvh {
 public:
  void build(const Mesh& mesh);
  bool empty() const { return nodes_.empty(); }

  // Nearest point on the mesh surface to q.
  ClosestPoint closest(const Eigen::Vector3d& q) const;

 private:
  struct Node {
    Eigen::AlignedBox3d box;
    int left = -1, right = -1;  // child node indices (-1 if leaf)
    int start = 0, count = 0;   // range into faceOrder_ for leaves
  };
  int buildNode(int start, int count);

  Eigen::MatrixXd V_;
  Eigen::MatrixXi F_;
  std::vector<int> faceOrder_;
  std::vector<Eigen::Vector3d> centroid_;
  std::vector<Node> nodes_;
};

}  // namespace ma
