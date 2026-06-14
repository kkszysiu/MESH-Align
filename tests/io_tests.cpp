#include "ma/IO.h"
#include "ma/Mesh.h"
#include "ma/MeshValidation.h"
#include "ma/PrimitiveMeshes.h"

#include <cmath>
#include <cstdio>
#include <string>

static int g_failures = 0;
#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                                        \
    }                                                                      \
  } while (0)

static bool dimsMatch(const ma::Mesh& a, const ma::Mesh& b, double tol) {
  Eigen::Vector3d da = a.bboxMax() - a.bboxMin();
  Eigen::Vector3d db = b.bboxMax() - b.bboxMin();
  return (da - db).cwiseAbs().maxCoeff() < tol;
}

int main() {
  const ma::Mesh box = ma::makeBox(10, 20, 30);

  // --- STL round trip: welds the flat box to 8 unique verts -> watertight ---
  {
    const std::string p = "rt_box.stl";
    ma::io::saveMesh(p, box);
    ma::Mesh r = ma::io::loadMesh(p);
    CHECK(r.numFaces() == 12);
    CHECK(r.numVertices() == 8);
    CHECK(dimsMatch(box, r, 1e-3));
    ma::ValidationReport v = ma::validate(r);
    CHECK(v.components == 1);
    CHECK(v.boundaryEdges == 0);
    CHECK(v.nonManifoldEdges == 0);
    CHECK(v.isWatertight);
  }

  // --- OBJ round trip: preserves geometry (unwelded) ---
  {
    const std::string p = "rt_box.obj";
    ma::io::saveMesh(p, box);
    ma::Mesh r = ma::io::loadMesh(p);
    CHECK(r.numFaces() == 12);
    CHECK(dimsMatch(box, r, 1e-3));
  }

  // --- PLY round trip ---
  {
    const std::string p = "rt_box.ply";
    ma::io::saveMesh(p, box);
    ma::Mesh r = ma::io::loadMesh(p);
    CHECK(r.numFaces() == 12);
    CHECK(dimsMatch(box, r, 1e-3));
  }

  // --- transform application ---
  {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = 100.0;  // translate +100 in X
    ma::Mesh t = ma::io::transformed(box, T);
    CHECK(std::abs(t.center().x() - 100.0) < 1e-9);
    CHECK(std::abs(t.center().y()) < 1e-9);
  }

  // --- extension parsing ---
  CHECK(ma::io::extensionOf("/a/b/Part.STL") == "stl");
  CHECK(ma::io::extensionOf("noext") == "");

  if (g_failures == 0) {
    std::printf("All IO tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d IO test(s) failed.\n", g_failures);
  return 1;
}
