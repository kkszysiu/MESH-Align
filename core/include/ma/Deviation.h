#pragma once
#include "ma/Bvh.h"
#include "ma/Mesh.h"
#include "ma/Types.h"

namespace ma {

struct IcpResult {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();  // scan -> reference
  double rms = 0.0;
  int iterations = 0;
};

// Best-fit the scan onto a nominal reference (PCA init + point-to-plane ICP).
IcpResult icpToReference(const Mesh& scan, const Mesh& reference, const Bvh& refBvh,
                         int maxIters = 50, bool pcaInit = true);

// Per-vertex signed distance from each scan vertex to the reference surface
// (+ = outside / material excess). `scanToRef` is applied to scan vertices first.
DeviationField deviationToReference(const Mesh& scan, const Mesh& reference, const Bvh& refBvh,
                                    const Eigen::Matrix4d& scanToRef, const ToleranceBands& bands);

// Per-vertex signed distance to a single plane (mode b: vs a fitted datum).
DeviationField deviationToPlane(const Mesh& scan, const Eigen::Vector3d& point,
                                const Eigen::Vector3d& normal, const ToleranceBands& bands);

struct PlaneResidual {
  double rms = 0.0;
  double maxAbs = 0.0;
};

// Flatness of a detected plane: RMS / max absolute distance of the inlier faces'
// vertices to the fitted plane (point, unit normal). Empty inliers -> zeros.
PlaneResidual planeResidual(const Mesh& mesh, const Eigen::Vector3d& point,
                            const Eigen::Vector3d& normal, const std::vector<int>& inlierFaces);

}  // namespace ma
