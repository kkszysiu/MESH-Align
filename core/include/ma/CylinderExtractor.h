#pragma once
#include "ma/Geometry.h"
#include "ma/Mesh.h"
#include "ma/Types.h"

#include <vector>

namespace ma {

struct CylinderExtractionParams {
  double radialToleranceFrac = 0.01;  // |dist-r| band as fraction of bbox diagonal
  int ransacIterations = 3000;
  int maxCylinders = 6;
  double minInlierFrac = 0.02;  // of candidate faces
  int minInliers = 24;
};

// RANSAC cylinder/bore detection over faces not in `skip` (skip.size()==#F, or
// empty to consider all). Deterministic (fixed RNG seed). Sorted by confidence.
std::vector<DetectedCylinder> extractCylinders(const Mesh& mesh, const FaceData& fd,
                                               const std::vector<char>& skip,
                                               const CylinderExtractionParams& params = {});

}  // namespace ma
