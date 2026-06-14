#pragma once
#include "ma/Geometry.h"
#include "ma/Mesh.h"
#include "ma/Types.h"

#include <vector>

namespace ma {

struct PlaneExtractionParams {
  double angleToleranceDeg = 12.0;   // max normal deviation when growing
  double distToleranceFrac = 0.004;  // band thickness as fraction of bbox diagonal
  double minAreaFrac = 0.01;         // ignore regions smaller than this fraction
  int maxPlanes = 16;
};

// Region-growing planar segmentation with area-weighted PCA refit.
// Deterministic (seeds by largest remaining face area). Results are sorted by
// confidence, descending.
std::vector<DetectedPlane> extractPlanes(const Mesh& mesh, const FaceData& fd,
                                         const std::vector<std::array<int, 3>>& adjacency,
                                         const PlaneExtractionParams& params = {});

}  // namespace ma
