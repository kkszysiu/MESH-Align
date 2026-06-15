#include "ma/AnalysisPipeline.h"
#include "ma/CylinderExtractor.h"
#include "ma/Geometry.h"
#include "ma/IO.h"
#include "ma/Mesh.h"
#include "ma/PlaneExtractor.h"
#include "ma/PrimitiveMeshes.h"
#include "ma/SymmetryDetector.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>

static int g_failures = 0;
#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                                        \
    }                                                                      \
  } while (0)

// Is every face normal aligned to a coordinate axis (axis-aligned box)?
static bool axisAligned(const ma::Mesh& m, double tol) {
  const ma::FaceData fd = ma::computeFaceData(m);
  for (Eigen::Index f = 0; f < fd.normal.rows(); ++f) {
    const double mx = fd.normal.row(f).cwiseAbs().maxCoeff();
    if (mx < 1.0 - tol) return false;
  }
  return true;
}

int main() {
  // ---- planes on a box: 6 faces ----
  {
    ma::Mesh box = ma::makeBox(40, 20, 10);
    ma::FaceData fd = ma::computeFaceData(box);
    auto adj = ma::triangleAdjacency(box);
    auto planes = ma::extractPlanes(box, fd, adj);
    CHECK(planes.size() == 6);
    double maxArea = 0;
    for (auto& p : planes) maxArea = std::max(maxArea, p.area);
    CHECK(std::abs(maxArea - 40.0 * 20.0) < 1.0);  // largest face = 800
  }

  // ---- symmetry of a box is high ----
  {
    ma::Mesh box = ma::makeBox(40, 20, 10);
    auto s = ma::detectSymmetry(box);
    CHECK(s.score > 0.8);
  }

  // ---- cylinder axis + radius ----
  {
    ma::Mesh cyl = ma::makeCylinder(10.0, 40.0, 64);
    ma::FaceData fd = ma::computeFaceData(cyl);
    std::vector<char> none;
    auto cs = ma::extractCylinders(cyl, fd, none);
    CHECK(!cs.empty());
    if (!cs.empty()) {
      CHECK(std::abs(cs[0].radius - 10.0) < 0.5);
      CHECK(std::abs(cs[0].axisDir.normalized().dot(Eigen::Vector3d::UnitZ())) > 0.99);
      CHECK(cs[0].length > 35.0 && cs[0].length < 41.0);  // wall spans ~40mm
    }
  }

  // ---- end-to-end: a tilted box gets squared up ----
  {
    ma::Mesh box = ma::makeBox(40, 20, 10);
    Eigen::Matrix4d Rt = Eigen::Matrix4d::Identity();
    Rt.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    ma::Mesh tilted = ma::io::transformed(box, Rt);
    CHECK(!axisAligned(tilted, 1e-3));  // sanity: it really is tilted

    ma::AnalysisResult res = ma::analyze(tilted);
    CHECK(res.orientation.confidence > 0.5);

    // the Top-role plane is classified Primary; all planes carry a source tag
    int primaries = 0;
    for (const auto& p : res.planes) {
      if (p.role == ma::DatumRole::Top) {
        CHECK(p.tier == "Primary");
        ++primaries;
      }
      CHECK(!p.source.empty());
    }
    CHECK(primaries == 1);

    // PCA elongation of a 40x20x10 box ~ 16 : 4 : 1 (rotation-invariant)
    const Eigen::Vector3d pr = res.orientation.pcaRatio;
    CHECK(std::abs(pr.x() - 16.0) < 1.0);
    CHECK(std::abs(pr.y() - 4.0) < 0.5);
    CHECK(std::abs(pr.z() - 1.0) < 1e-6);
    ma::Mesh aligned = ma::io::transformed(tilted, res.orientation.transform);
    CHECK(axisAligned(aligned, 1e-2));

    // sorted bbox dims preserved (== 10,20,40)
    Eigen::Vector3d d = aligned.bboxMax() - aligned.bboxMin();
    std::sort(d.data(), d.data() + 3);
    CHECK(std::abs(d(0) - 10) < 0.2);
    CHECK(std::abs(d(1) - 20) < 0.2);
    CHECK(std::abs(d(2) - 40) < 0.2);
  }

  if (g_failures == 0) {
    std::printf("All analysis tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d analysis test(s) failed.\n", g_failures);
  return 1;
}
