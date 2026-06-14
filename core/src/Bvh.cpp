#include "ma/Bvh.h"

#include <algorithm>
#include <limits>

namespace ma {

namespace {
// Closest point on triangle (a,b,c) to p (Ericson, Real-Time Collision Detection).
Eigen::Vector3d closestOnTriangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
  const Eigen::Vector3d ab = b - a, ac = c - a, ap = p - a;
  const double d1 = ab.dot(ap), d2 = ac.dot(ap);
  if (d1 <= 0 && d2 <= 0) return a;
  const Eigen::Vector3d bp = p - b;
  const double d3 = ab.dot(bp), d4 = ac.dot(bp);
  if (d3 >= 0 && d4 <= d3) return b;
  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + (d1 / (d1 - d3)) * ab;
  const Eigen::Vector3d cp = p - c;
  const double d5 = ab.dot(cp), d6 = ac.dot(cp);
  if (d6 >= 0 && d5 <= d6) return c;
  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + (d2 / (d2 - d6)) * ac;
  const double va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
    return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom, w = vc * denom;
  return a + ab * v + ac * w;
}
}  // namespace

int Bvh::buildNode(int start, int count) {
  const int nodeIdx = static_cast<int>(nodes_.size());
  nodes_.emplace_back();
  Eigen::AlignedBox3d box;
  for (int i = start; i < start + count; ++i) {
    const int f = faceOrder_[static_cast<size_t>(i)];
    for (int k = 0; k < 3; ++k) box.extend(V_.row(F_(f, k)).transpose());
  }

  if (count <= 4) {
    nodes_[static_cast<size_t>(nodeIdx)].box = box;
    nodes_[static_cast<size_t>(nodeIdx)].start = start;
    nodes_[static_cast<size_t>(nodeIdx)].count = count;
    return nodeIdx;
  }

  // split along the largest box dimension at the median centroid
  int axis = 0;
  box.sizes().maxCoeff(&axis);
  const int mid = start + count / 2;
  std::nth_element(faceOrder_.begin() + start, faceOrder_.begin() + mid,
                   faceOrder_.begin() + start + count, [&](int fa, int fb) {
                     return centroid_[static_cast<size_t>(fa)](axis) <
                            centroid_[static_cast<size_t>(fb)](axis);
                   });
  const int left = buildNode(start, mid - start);
  const int right = buildNode(mid, start + count - mid);
  nodes_[static_cast<size_t>(nodeIdx)].box = box;
  nodes_[static_cast<size_t>(nodeIdx)].left = left;
  nodes_[static_cast<size_t>(nodeIdx)].right = right;
  return nodeIdx;
}

void Bvh::build(const Mesh& mesh) {
  V_ = mesh.V;
  F_ = mesh.F;
  nodes_.clear();
  const int nf = static_cast<int>(F_.rows());
  faceOrder_.resize(static_cast<size_t>(nf));
  centroid_.resize(static_cast<size_t>(nf));
  for (int f = 0; f < nf; ++f) {
    faceOrder_[static_cast<size_t>(f)] = f;
    centroid_[static_cast<size_t>(f)] =
        (V_.row(F_(f, 0)) + V_.row(F_(f, 1)) + V_.row(F_(f, 2))).transpose() / 3.0;
  }
  if (nf > 0) {
    nodes_.reserve(static_cast<size_t>(2 * nf));
    buildNode(0, nf);
  }
}

ClosestPoint Bvh::closest(const Eigen::Vector3d& q) const {
  ClosestPoint best;
  best.distance = std::numeric_limits<double>::infinity();
  if (nodes_.empty()) return best;

  // explicit stack of node indices
  std::vector<int> stack;
  stack.reserve(64);
  stack.push_back(0);
  while (!stack.empty()) {
    const Node& n = nodes_[static_cast<size_t>(stack.back())];
    stack.pop_back();
    if (n.box.squaredExteriorDistance(q) > best.distance * best.distance) continue;

    if (n.left < 0) {  // leaf
      for (int i = n.start; i < n.start + n.count; ++i) {
        const int f = faceOrder_[static_cast<size_t>(i)];
        const Eigen::Vector3d cp =
            closestOnTriangle(q, V_.row(F_(f, 0)).transpose(), V_.row(F_(f, 1)).transpose(),
                              V_.row(F_(f, 2)).transpose());
        const double d = (cp - q).norm();
        if (d < best.distance) {
          best.distance = d;
          best.point = cp;
          best.face = f;
        }
      }
    } else {
      stack.push_back(n.left);
      stack.push_back(n.right);
    }
  }
  return best;
}

}  // namespace ma
