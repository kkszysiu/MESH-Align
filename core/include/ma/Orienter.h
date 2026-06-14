#pragma once
#include "ma/Mesh.h"
#include "ma/Types.h"

#include <vector>

namespace ma {

// Decide a Top/Front/Right datum frame from the detected features and return the
// part->datum transform plus a confidence and a human-readable method string.
OrientationResult orient(const Mesh& mesh, std::vector<DetectedPlane>& planes,
                         const std::vector<DetectedCylinder>& cylinders,
                         const SymmetryPlane& symmetry);

// Build an orientation from explicit Top (Z) and Right (X) direction hints (part
// space) and an origin. X is orthonormalised against Z. Used by manual overrides.
OrientationResult orientFromAxes(const Mesh& mesh, const Eigen::Vector3d& Zhint,
                                 const Eigen::Vector3d& Xhint, const Eigen::Vector3d& origin,
                                 const std::string& method, double confidence);

}  // namespace ma
