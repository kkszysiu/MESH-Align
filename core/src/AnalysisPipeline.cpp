#include "ma/AnalysisPipeline.h"

#include "ma/CylinderExtractor.h"
#include "ma/Geometry.h"
#include "ma/MeshValidation.h"
#include "ma/Orienter.h"
#include "ma/PlaneExtractor.h"
#include "ma/SymmetryDetector.h"

#include <cmath>

namespace ma {

namespace {
bool cancelled(const std::atomic<bool>* c) { return c && c->load(); }

void report(const ProgressFn& fn, Stage s, float pct, const std::string& msg) {
  if (fn) fn(s, pct, msg);
}
}  // namespace

AnalysisResult analyze(const Mesh& mesh, const ProgressFn& progress,
                       const std::atomic<bool>* cancel, const PipelineParams& params) {
  AnalysisResult result;
  if (mesh.empty()) return result;

  report(progress, Stage::Validation, 0.05f, "Validating mesh");
  result.validation = validate(mesh);
  if (cancelled(cancel)) return result;

  report(progress, Stage::Normals, 0.15f, "Computing face data");
  const FaceData fd = computeFaceData(mesh);
  const auto adjacency = triangleAdjacency(mesh);
  if (cancelled(cancel)) return result;

  std::vector<char> planeFaces(static_cast<size_t>(mesh.F.rows()), 0);
  if (params.detectPlanes) {
    report(progress, Stage::Planes, 0.30f, "Extracting planes");
    result.planes = extractPlanes(mesh, fd, adjacency);
    for (const auto& p : result.planes)
      for (int f : p.inlierFaces) planeFaces[static_cast<size_t>(f)] = 1;
  }
  if (cancelled(cancel)) return result;

  if (params.detectCylinders) {
    report(progress, Stage::Cylinders, 0.55f, "Detecting bores");
    result.cylinders = extractCylinders(mesh, fd, planeFaces);
  }
  if (cancelled(cancel)) return result;

  SymmetryPlane sym;
  if (params.detectSymmetry) {
    report(progress, Stage::Symmetry, 0.75f, "Detecting symmetry");
    sym = detectSymmetry(mesh);
    result.symmetry = {sym};
  }
  if (cancelled(cancel)) return result;

  report(progress, Stage::Orientation, 0.90f, "Assigning datums");
  result.orientation = orient(mesh, result.planes, result.cylinders, sym);

  // Classify candidate flats that sit concentric with a bore as "bore-seat".
  for (auto& p : result.planes) {
    if (p.role != DatumRole::None) continue;  // keep datum tiers
    for (const auto& c : result.cylinders) {
      if (std::abs(p.normal.normalized().dot(c.axisDir)) < 0.9) continue;  // not face-on
      const Eigen::Vector3d w = p.point - c.axisPoint;
      const Eigen::Vector3d radial = w - w.dot(c.axisDir) * c.axisDir;
      if (radial.norm() < 2.0 * c.radius) { p.source = "bore-seat"; break; }
    }
  }

  report(progress, Stage::Done, 1.0f, "Done");
  return result;
}

}  // namespace ma
