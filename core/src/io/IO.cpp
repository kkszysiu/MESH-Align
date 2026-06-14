#include "ma/IO.h"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>

namespace ma::io {

std::string extensionOf(const std::string& path) {
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos) return "";
  std::string ext = path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext;
}

Mesh loadMesh(const std::string& path) {
  const std::string ext = extensionOf(path);
  Mesh m;
  if (ext == "stl") {
    m = loadSTL(path);
  } else if (ext == "obj") {
    m = loadOBJ(path);
  } else if (ext == "ply") {
    m = loadPLY(path);
  } else {
    throw IOError("Unsupported mesh format: ." + ext);
  }
  if (m.empty()) throw IOError("Mesh has no geometry: " + path);
  // name = filename stem
  auto slash = path.find_last_of("/\\");
  m.name = path.substr(slash == std::string::npos ? 0 : slash + 1);
  return m;
}

Mesh transformed(const Mesh& mesh, const Eigen::Matrix4d& T) {
  Mesh out = mesh;
  const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
  const Eigen::Vector3d t = T.topRightCorner<3, 1>();
  for (Eigen::Index i = 0; i < out.V.rows(); ++i)
    out.V.row(i) = (R * mesh.V.row(i).transpose() + t).transpose();
  if (out.VN.rows() == out.V.rows()) {
    const Eigen::Matrix3d N = R.inverse().transpose();
    for (Eigen::Index i = 0; i < out.VN.rows(); ++i)
      out.VN.row(i) = (N * mesh.VN.row(i).transpose()).normalized().transpose();
  }
  return out;
}

namespace {
void writeOBJ(const std::string& path, const Mesh& m) {
  std::ofstream f(path);
  if (!f) throw IOError("Cannot open for writing: " + path);
  f << "# MESH-Align export\n";
  for (Eigen::Index i = 0; i < m.V.rows(); ++i)
    f << "v " << m.V(i, 0) << ' ' << m.V(i, 1) << ' ' << m.V(i, 2) << '\n';
  for (Eigen::Index i = 0; i < m.F.rows(); ++i)
    f << "f " << m.F(i, 0) + 1 << ' ' << m.F(i, 1) + 1 << ' ' << m.F(i, 2) + 1 << '\n';
}

void writeSTLBinary(const std::string& path, const Mesh& m) {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw IOError("Cannot open for writing: " + path);
  char header[80] = {0};
  std::snprintf(header, sizeof(header), "MESH-Align binary STL");
  f.write(header, 80);
  uint32_t ntri = static_cast<uint32_t>(m.F.rows());
  f.write(reinterpret_cast<const char*>(&ntri), 4);
  for (Eigen::Index i = 0; i < m.F.rows(); ++i) {
    const Eigen::Vector3d a = m.V.row(m.F(i, 0));
    const Eigen::Vector3d b = m.V.row(m.F(i, 1));
    const Eigen::Vector3d c = m.V.row(m.F(i, 2));
    Eigen::Vector3f n = ((b - a).cross(c - a)).normalized().cast<float>();
    auto wf = [&](const Eigen::Vector3f& v) { f.write(reinterpret_cast<const char*>(v.data()), 12); };
    wf(n);
    wf(a.cast<float>());
    wf(b.cast<float>());
    wf(c.cast<float>());
    uint16_t attr = 0;
    f.write(reinterpret_cast<const char*>(&attr), 2);
  }
}

}  // namespace

void saveMesh(const std::string& path, const Mesh& mesh) {
  const std::string ext = extensionOf(path);
  if (ext == "obj")
    writeOBJ(path, mesh);
  else if (ext == "stl")
    writeSTLBinary(path, mesh);
  else if (ext == "ply")
    savePLYBinary(path, mesh);
  else
    throw IOError("Unsupported export format: ." + ext);
}

void saveTransform(const std::string& path, const Eigen::Matrix4d& T) {
  std::ofstream f(path);
  if (!f) throw IOError("Cannot open for writing: " + path);
  f << "# MESH-Align part->datum transform (row-major 4x4)\n";
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) f << T(r, c) << (c == 3 ? '\n' : ' ');
  }
}

}  // namespace ma::io
