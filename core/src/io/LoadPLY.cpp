#include "ma/IO.h"

#define TINYPLY_IMPLEMENTATION
#include <tinyply.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace ma::io {

namespace {

// --- binary PLY via tinyply (its binary path is reliable) --------------------
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

Mesh loadBinaryPLY(std::ifstream& stream) {
  tinyply::PlyFile file;
  file.parse_header(stream);

  std::shared_ptr<tinyply::PlyData> verts, faces;
  try {
    verts = file.request_properties_from_element("vertex", {"x", "y", "z"});
  } catch (const std::exception&) {
    throw IOError("PLY missing vertex x/y/z");
  }
  try {
    faces = file.request_properties_from_element("face", {"vertex_indices"}, 3);
  } catch (const std::exception&) {
    try {
      faces = file.request_properties_from_element("face", {"vertex_index"}, 3);
    } catch (const std::exception&) {
      throw IOError("PLY missing triangulated face list");
    }
  }
  file.read(stream);

  Mesh m;
  copyVertices(*verts, m.V);
  if (faces) copyFaces(*faces, m.F);
  return m;
}

// --- ascii PLY: small hand-rolled reader (tinyply 2.3.4 ascii is buggy) ------
Mesh loadAsciiPLY(std::ifstream& f) {
  std::string line;
  int vCount = 0, fCount = 0;
  int vProps = 0, xi = -1, yi = -1, zi = -1;
  std::string element;
  bool haveFaceList = false;

  while (std::getline(f, line)) {
    std::istringstream ls(line);
    std::string tok;
    ls >> tok;
    if (tok == "end_header") break;
    if (tok == "element") {
      ls >> element;
      if (element == "vertex") ls >> vCount;
      else if (element == "face") ls >> fCount;
    } else if (tok == "property") {
      std::string a;
      ls >> a;
      if (a == "list") {
        if (element == "face") haveFaceList = true;
      } else if (element == "vertex") {
        std::string pname;
        ls >> pname;
        if (pname == "x") xi = vProps;
        else if (pname == "y") yi = vProps;
        else if (pname == "z") zi = vProps;
        ++vProps;
      }
    }
  }
  if (xi < 0 || yi < 0 || zi < 0) throw IOError("ascii PLY missing x/y/z");
  if (!haveFaceList) throw IOError("ascii PLY missing face list");

  Mesh m;
  m.V.resize(vCount, 3);
  for (int i = 0; i < vCount; ++i) {
    std::vector<double> row(static_cast<size_t>(vProps));
    for (int p = 0; p < vProps; ++p) f >> row[static_cast<size_t>(p)];
    m.V.row(i) << row[xi], row[yi], row[zi];
  }

  std::vector<Eigen::Vector3i> tris;
  for (int i = 0; i < fCount; ++i) {
    int n = 0;
    f >> n;
    std::vector<int> idx(static_cast<size_t>(n));
    for (int k = 0; k < n; ++k) f >> idx[static_cast<size_t>(k)];
    for (int k = 1; k + 1 < n; ++k)  // fan triangulation
      tris.emplace_back(idx[0], idx[static_cast<size_t>(k)], idx[static_cast<size_t>(k + 1)]);
  }
  m.F.resize(static_cast<Eigen::Index>(tris.size()), 3);
  for (size_t i = 0; i < tris.size(); ++i)
    m.F.row(static_cast<Eigen::Index>(i)) = tris[i].transpose();
  return m;
}

}  // namespace

Mesh loadPLY(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw IOError("Cannot open: " + path);

  // Peek the format line in the header.
  std::string magic, fmt;
  std::getline(stream, magic);  // "ply"
  std::string line;
  bool ascii = false, known = false;
  std::streampos afterPly = stream.tellg();
  while (std::getline(stream, line)) {
    if (line.rfind("format", 0) == 0) {
      ascii = line.find("ascii") != std::string::npos;
      known = true;
      break;
    }
    if (line.rfind("end_header", 0) == 0) break;
  }
  if (!known) throw IOError("PLY missing format line");

  if (ascii) {
    stream.clear();
    stream.seekg(afterPly);  // re-parse header in the ascii reader
    return loadAsciiPLY(stream);
  }
  stream.clear();
  stream.seekg(0);
  return loadBinaryPLY(stream);
}

// Binary little-endian writer — guaranteed readable by tinyply.
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
