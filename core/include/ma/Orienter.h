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

}  // namespace ma
