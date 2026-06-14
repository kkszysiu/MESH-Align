#include "ma/Geometry.h"
#include "ma/IO.h"
#include "ma/Orienter.h"
#include "ma/Picking.h"
#include "ma/PrimitiveMeshes.h"

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

static bool axisAligned(const ma::Mesh& m, double tol) {
  ma::FaceData fd = ma::computeFaceData(m);
  for (Eigen::Index f = 0; f < fd.normal.rows(); ++f)
    if (fd.normal.row(f).cwiseAbs().maxCoeff() < 1.0 - tol) return false;
  return true;
}

int main() {
  ma::Mesh box = ma::makeBox(40, 20, 10);  // spans x[-20,20] y[-10,10] z[-5,5]

  // ---- raycast straight down onto the +Z face hits at z=+5 ----
  {
    ma::RayHit h = ma::raycast(box, Eigen::Vector3d(0, 0, 100), Eigen::Vector3d(0, 0, -1));
    CHECK(h.hit);
    CHECK(std::abs(h.point.z() - 5.0) < 1e-6);
    CHECK(std::abs(h.point.x()) < 1e-6 && std::abs(h.point.y()) < 1e-6);
  }
  // ---- ray that misses returns no hit ----
  {
    ma::RayHit h = ma::raycast(box, Eigen::Vector3d(1000, 1000, 100), Eigen::Vector3d(0, 0, -1));
    CHECK(!h.hit);
  }

  // ---- smooth-region grow on a cylinder wall captures the curved surface ----
  {
    ma::Mesh cyl = ma::makeCylinder(10, 40, 48);
    ma::FaceData fd = ma::computeFaceData(cyl);
    auto adj = ma::triangleAdjacency(cyl);
    // a side-wall face points radially (small |z|); find one
    int seed = -1;
    for (Eigen::Index f = 0; f < fd.normal.rows(); ++f)
      if (std::abs(fd.normal(f, 2)) < 0.1) { seed = static_cast<int>(f); break; }
    CHECK(seed >= 0);
    auto region = ma::growSmoothRegion(cyl, fd, adj, seed, 20.0);
    // wall = 48 segments * 2 tris = 96; should grow most of it, not the caps
    CHECK(region.size() > 80);
    CHECK(region.size() < 110);
  }

  // ---- orientFromAxes squares up a tilted box ----
  {
    Eigen::Matrix4d Rt = Eigen::Matrix4d::Identity();
    Rt.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(0.5, Eigen::Vector3d(2, 1, 1).normalized()).toRotationMatrix();
    ma::Mesh tilted = ma::io::transformed(box, Rt);
    // the +Z face normal (40x20 face) in tilted space:
    Eigen::Vector3d Z = Rt.topLeftCorner<3, 3>() * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d X = Rt.topLeftCorner<3, 3>() * Eigen::Vector3d(1, 0, 0);
    auto res = ma::orientFromAxes(tilted, Z, X, tilted.centroid(), "manual", 1.0);
    ma::Mesh aligned = ma::io::transformed(tilted, res.transform);
    CHECK(axisAligned(aligned, 1e-3));
  }

  if (g_failures == 0) {
    std::printf("All picking tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d picking test(s) failed.\n", g_failures);
  return 1;
}
