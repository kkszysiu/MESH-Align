#pragma once
#include "ma/Mesh.h"
#include "ma/Types.h"

namespace ma {

// Detect the best reflective-symmetry plane by testing the mesh's principal
// axes and scoring how well a mirrored copy matches the original (nearest-
// neighbour distance). score in [0,1]; higher = more symmetric.
SymmetryPlane detectSymmetry(const Mesh& mesh, int maxSamples = 3000);

}  // namespace ma
