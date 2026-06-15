#include "ma/IO.h"

#define TINYPLY_IMPLEMENTATION
#include <tinyply.h>

#include <cstdint>
#include <cstring>
#include <fstream>

namespace ma::io {

namespace {
// Copy a tinyply vertex buffer (float32 or float64) into V.
void copyVertices(tinyply::PlyData& data, Eigen::MatrixXd& V) {
  const size_t n = data.count;
  V.resize(static_cast<Eigen::Index>(n), 3);
  if (data.t == tinyply::Type::FLOAT32) {
    const float* p = reinterpret_cast<const float*>(data.buffer.get());
    for (size_t i = 0; i < n; ++i)
      V.row(static_cast<Eigen::Index>(i)) << p[3 * i], p[3 * i + 1], p[3 * i + 2];
  } else if (data.t == tinyply::Type::FLOAT64) {
    const double* p = reinterpret_cast<const double*>(data.buffer.get());
    for (size_t i = 0; i < n; ++i)
      V.row(static_cast<Eigen::Index>(i)) << p[3 * i], p[3 * i + 1], p[3 * i + 2];
  } else {
    throw IOError("PLY vertex coordinates must be float or double");
  }
}

// Copy a triangle index buffer (int16/32, signed or unsigned) into F (3 per face).
void copyFaces(tinyply::PlyData& data, Eigen::MatrixXi& F) {
  const size_t n = data.count;
  F.resize(static_cast<Eigen::Index>(n), 3);
  const uint8_t* base = data.buffer.get();
  const size_t stride =
      (data.t == tinyply::Type::INT16 || data.t == tinyply::Type::UINT16) ? 2 : 4;
  for (size_t i = 0; i < n; ++i) {
    int idx[3];
    for (int k = 0; k < 3; ++k) {
      int32_t v = 0;
      std::memcpy(&v, base + (3 * i + k) * stride, stride == 2 ? 2 : 4);
      idx[k] = v;
    }
    F.row(static_cast<Eigen::Index>(i)) << idx[0], idx[1], idx[2];
  }
}
}  // namespace

// tinyply 3.0 handles both ASCII and binary; we triangulate-assume face lists.
Mesh loadPLY(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw IOError("Cannot open: " + path);

  tinyply::PlyFile file;
  try {
    file.parse_header(stream);
  } catch (const std::exception& e) {
    throw IOError(std::string("PLY header parse failed: ") + e.what());
  }

  std::shared_ptr<tinyply::PlyData> verts, faces;
  try {
    verts = file.request_properties_from_element("vertex", {"x", "y", "z"});
  } catch (const std::exception&) {
    throw IOError("PLY missing vertex x/y/z");
  }
  // Face index property is named "vertex_indices" or "vertex_index"; assume tris.
  try {
    faces = file.request_properties_from_element("face", {"vertex_indices"}, 3);
  } catch (const std::exception&) {
    try {
      faces = file.request_properties_from_element("face", {"vertex_index"}, 3);
    } catch (const std::exception&) {
      throw IOError("PLY missing triangulated face list");
    }
  }

  try {
    file.read(stream);
  } catch (const std::exception& e) {
    throw IOError(std::string("PLY read failed: ") + e.what());
  }

  Mesh m;
  copyVertices(*verts, m.V);
  if (faces) copyFaces(*faces, m.F);
  return m;
}

// Binary little-endian writer — small, deterministic, readable by tinyply.
void savePLYBinary(const std::string& path, const Mesh& m) {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw IOError("Cannot open for writing: " + path);
  f << "ply\nformat binary_little_endian 1.0\ncomment MESH-Align export\n";
  f << "element vertex " << m.V.rows() << "\n";
  f << "property float x\nproperty float y\nproperty float z\n";
  f << "element face " << m.F.rows() << "\n";
  f << "property list uchar int vertex_indices\nend_header\n";
  for (Eigen::Index i = 0; i < m.V.rows(); ++i) {
    float xyz[3] = {static_cast<float>(m.V(i, 0)), static_cast<float>(m.V(i, 1)),
                    static_cast<float>(m.V(i, 2))};
    f.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
  for (Eigen::Index i = 0; i < m.F.rows(); ++i) {
    uint8_t n = 3;
    int32_t idx[3] = {m.F(i, 0), m.F(i, 1), m.F(i, 2)};
    f.write(reinterpret_cast<const char*>(&n), 1);
    f.write(reinterpret_cast<const char*>(idx), sizeof(idx));
  }
}

}  // namespace ma::io
