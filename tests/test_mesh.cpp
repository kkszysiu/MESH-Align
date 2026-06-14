#include "ma/Mesh.h"
#include "ma/PrimitiveMeshes.h"
#include "ma/Types.h"

#include <cmath>
#include <cstdio>
#include <string>

static int g_failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                                     \
    }                                                                   \
  } while (0)

int main() {
  // makeBox(10,20,30): 6 quad faces -> 12 triangles, 24 verts (flat-shaded).
  ma::Mesh box = ma::makeBox(10, 20, 30);
  CHECK(box.numFaces() == 12);
  CHECK(box.numVertices() == 24);
  CHECK(std::abs(box.surfaceArea() - 2.0 * (10 * 20 + 20 * 30 + 10 * 30)) < 1e-6);
  CHECK(std::abs(std::abs(box.volume()) - 10.0 * 20.0 * 30.0) < 1e-6);

  Eigen::Vector3d d = box.bboxMax() - box.bboxMin();
  CHECK(std::abs(d.x() - 10) < 1e-9);
  CHECK(std::abs(d.y() - 20) < 1e-9);
  CHECK(std::abs(d.z() - 30) < 1e-9);
  CHECK(box.center().norm() < 1e-9);  // centred at origin

  // Demo part = 3 boxes -> 36 triangles.
  ma::Mesh part = ma::makeDemoPart();
  CHECK(!part.empty());
  CHECK(part.numFaces() == 36);
  CHECK(part.diagonal() > 0.0);

  // Stage naming is wired.
  CHECK(std::string(ma::stageName(ma::Stage::Orientation)) == "Auto-orientation");

  if (g_failures == 0) {
    std::printf("All core tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d core test(s) failed.\n", g_failures);
  return 1;
}
