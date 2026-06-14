#include "ma/PlaneExtractor.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace ma {

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// Grow a planar region across adjacency from `seed`, given a reference plane
// (point0, normal0). Returns the set of inlier faces.
std::vector<int> grow(const FaceData& fd, const std::vector<std::array<int, 3>>& adj,
                      const std::vector<char>& assigned, int seed,
                      const Eigen::Vector3d& point0, const Eigen::Vector3d& normal0,
                      double cosTol, double distTol) {
  std::vector<int> region;
  std::vector<char> inRegion(fd.area.size(), 0);
  std::queue<int> q;
  q.push(seed);
  inRegion[static_cast<size_t>(seed)] = 1;
  while (!q.empty()) {
    const int f = q.front();
    q.pop();
    region.push_back(f);
    for (int nb : adj[static_cast<size_t>(f)]) {
      if (nb < 0 || assigned[static_cast<size_t>(nb)] || inRegion[static_cast<size_t>(nb)])
        continue;
      const Eigen::Vector3d nn = fd.normal.row(nb).transpose();
      if (nn.dot(normal0) < cosTol) continue;
      const Eigen::Vector3d cc = fd.centroid.row(nb).transpose();
      if (std::abs((cc - point0).dot(normal0)) > distTol) continue;
      inRegion[static_cast<size_t>(nb)] = 1;
      q.push(nb);
    }
  }
  return region;
}

void refit(const Mesh& mesh, const FaceData& fd, const std::vector<int>& region,
           Eigen::Vector3d& point, Eigen::Vector3d& normal, double& rms) {
  // Fit through inlier vertices (3 per face), area-weighted.
  Eigen::MatrixXd P(static_cast<Eigen::Index>(region.size()) * 3, 3);
  Eigen::VectorXd w(static_cast<Eigen::Index>(region.size()) * 3);
  Eigen::Index r = 0;
  for (int f : region) {
    const double a = fd.area(f) / 3.0;
    for (int k = 0; k < 3; ++k) {
      P.row(r) = mesh.V.row(mesh.F(f, k));
      w(r) = a;
      ++r;
    }
  }
  Eigen::Vector3d evals;
  weightedPCA(P, w, point, normal, evals);
  rms = std::sqrt(std::max(0.0, evals(0)));
  // Orient consistently with the region's area-weighted face normals.
  Eigen::Vector3d meanN = Eigen::Vector3d::Zero();
  for (int f : region) meanN += fd.area(f) * fd.normal.row(f).transpose();
  if (meanN.dot(normal) < 0) normal = -normal;
}
}  // namespace

std::vector<DetectedPlane> extractPlanes(const Mesh& mesh, const FaceData& fd,
                                         const std::vector<std::array<int, 3>>& adjacency,
                                         const PlaneExtractionParams& params) {
  const Eigen::Index nf = mesh.F.rows();
  const double diag = mesh.diagonal();
  const double cosTol = std::cos(params.angleToleranceDeg * kDeg2Rad);
  const double distTol = params.distToleranceFrac * diag;
  const double minArea = params.minAreaFrac * fd.totalArea;

  std::vector<char> assigned(static_cast<size_t>(nf), 0);
  std::vector<DetectedPlane> planes;

  for (int iter = 0; iter < params.maxPlanes; ++iter) {
    // Seed = largest unassigned face.
    int seed = -1;
    double bestArea = 0.0;
    for (Eigen::Index f = 0; f < nf; ++f) {
      if (assigned[static_cast<size_t>(f)]) continue;
      if (fd.area(f) > bestArea) {
        bestArea = fd.area(f);
        seed = static_cast<int>(f);
      }
    }
    if (seed < 0) break;
    // Even the largest remaining face is negligible: nothing useful left.
    if (bestArea < 1e-6 * fd.totalArea) break;

    Eigen::Vector3d point = fd.centroid.row(seed).transpose();
    Eigen::Vector3d normal = fd.normal.row(seed).transpose();

    // Grow, refit, grow once more with the refined plane.
    std::vector<int> region = grow(fd, adjacency, assigned, seed, point, normal, cosTol, distTol);
    double rms = 0.0;
    if (!region.empty()) {
      refit(mesh, fd, region, point, normal, rms);
      region = grow(fd, adjacency, assigned, seed, point, normal, cosTol, distTol);
      refit(mesh, fd, region, point, normal, rms);
    }

    double regionArea = 0.0;
    for (int f : region) regionArea += fd.area(f);
    for (int f : region) assigned[static_cast<size_t>(f)] = 1;

    if (regionArea < minArea || region.empty()) continue;  // consume, don't record

    DetectedPlane p;
    p.point = point;
    p.normal = normal;
    p.area = regionArea;
    p.inlierFaces = region;
    // confidence = planarity (residual vs band) blended with relative area.
    const double planarity = std::exp(-rms / std::max(distTol * 0.5, 1e-9));
    const double areaScore = std::min(1.0, (regionArea / fd.totalArea) / 0.15);
    p.confidence = std::clamp(0.6 * planarity + 0.4 * areaScore, 0.0, 1.0);
    p.label = "Face " + std::to_string(planes.size());
    planes.push_back(std::move(p));
  }

  std::sort(planes.begin(), planes.end(),
            [](const DetectedPlane& a, const DetectedPlane& b) { return a.confidence > b.confidence; });
  return planes;
}

}  // namespace ma
