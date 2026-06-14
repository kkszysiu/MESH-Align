#include "ma/Bvh.h"
#include "ma/Deviation.h"
#include "ma/IO.h"
#include "ma/Mesh.h"
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

int main() {
  ma::Mesh box = ma::makeBox(40, 20, 10);  // x[-20,20] y[-10,10] z[-5,5]
  ma::Bvh bvh;
  bvh.build(box);

  // ---- closest point on surface ----
  {
    auto cp = bvh.closest(Eigen::Vector3d(0, 0, 100));  // above +Z face
    CHECK(cp.face >= 0);
    CHECK(std::abs(cp.point.z() - 5.0) < 1e-6);
    CHECK(std::abs(cp.distance - 95.0) < 1e-6);
    auto inside = bvh.closest(Eigen::Vector3d(0, 0, 0));  // centre -> nearest face at 5
    CHECK(std::abs(inside.distance - 5.0) < 1e-6);
  }

  // ---- ICP recovers a known rigid offset ----
  {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(0.25, Eigen::Vector3d(0.3, 1, 0.2).normalized()).toRotationMatrix();
    M.topRightCorner<3, 1>() = Eigen::Vector3d(7, -3, 5);
    ma::Mesh moved = ma::io::transformed(box, M);  // "scan" displaced from reference

    auto icp = ma::icpToReference(moved, box, bvh);
    // applying icp.transform to the moved scan should land back on the box
    ma::Mesh aligned = ma::io::transformed(moved, icp.transform);
    ma::Bvh bb;
    bb.build(box);
    double maxd = 0;
    for (Eigen::Index i = 0; i < aligned.V.rows(); ++i)
      maxd = std::max(maxd, bb.closest(aligned.V.row(i).transpose()).distance);
    CHECK(maxd < 0.05);
    CHECK(icp.rms < 0.05);
  }

  // ---- signed distance: a scan 0.3mm proud of nominal reads ~+0.3 ----
  {
    Eigen::Matrix4d up = Eigen::Matrix4d::Identity();
    up(2, 3) = 0.3;  // shift scan +0.3 in Z
    ma::Mesh scan = ma::io::transformed(box, up);
    ma::ToleranceBands bands;
    bands.inTolMm = 0.1;
    bands.warnMm = 0.5;
    auto dev = ma::deviationToReference(scan, box, bvh, Eigen::Matrix4d::Identity(), bands);
    // top face vertices should be ~+0.3 outside
    CHECK(dev.maxMm > 0.25 && dev.maxMm < 0.35);
    CHECK(dev.minMm < -0.25);  // bottom face now 0.3 inside
  }

  // ---- deviation to a plane ----
  {
    auto dev = ma::deviationToPlane(box, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1), {});
    CHECK(std::abs(dev.maxMm - 5.0) < 1e-6);
    CHECK(std::abs(dev.minMm + 5.0) < 1e-6);
  }

  if (g_failures == 0) {
    std::printf("All deviation tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d deviation test(s) failed.\n", g_failures);
  return 1;
}
