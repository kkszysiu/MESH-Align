#include "ma/IO.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <unordered_map>
#include <vector>

namespace ma::io {

namespace {
struct Key {
  float x, y, z;
  bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct KeyHash {
  size_t operator()(const Key& k) const {
    auto h = [](float f) {
      uint32_t u;
      std::memcpy(&u, &f, 4);
      return std::hash<uint32_t>()(u);
    };
    return h(k.x) ^ (h(k.y) << 1) ^ (h(k.z) << 2);
  }
};

// Incrementally builds an indexed mesh from a stream of triangle corners,
// welding exactly-equal vertices.
class Welder {
 public:
  int add(float x, float y, float z) {
    Key k{x, y, z};
    auto it = map_.find(k);
    if (it != map_.end()) return it->second;
    int idx = static_cast<int>(verts_.size());
    verts_.push_back({x, y, z});
    map_.emplace(k, idx);
    return idx;
  }
  void tri(int a, int b, int c) { faces_.push_back({a, b, c}); }
  Mesh build() {
    Mesh m;
    m.V.resize(static_cast<Eigen::Index>(verts_.size()), 3);
    m.F.resize(static_cast<Eigen::Index>(faces_.size()), 3);
    for (size_t i = 0; i < verts_.size(); ++i)
      m.V.row(static_cast<Eigen::Index>(i)) << verts_[i][0], verts_[i][1], verts_[i][2];
    for (size_t i = 0; i < faces_.size(); ++i)
      m.F.row(static_cast<Eigen::Index>(i)) << faces_[i][0], faces_[i][1], faces_[i][2];
    return m;
  }

 private:
  std::unordered_map<Key, int, KeyHash> map_;
  std::vector<std::array<float, 3>> verts_;
  std::vector<std::array<int, 3>> faces_;
};

Mesh loadBinarySTL(std::ifstream& f, uint32_t ntri) {
  Welder w;
  for (uint32_t t = 0; t < ntri; ++t) {
    float buf[12];
    f.seekg(12, std::ios::cur);  // skip per-facet normal
    f.read(reinterpret_cast<char*>(buf), 36);
    if (!f) throw IOError("Truncated binary STL");
    int a = w.add(buf[0], buf[1], buf[2]);
    int b = w.add(buf[3], buf[4], buf[5]);
    int c = w.add(buf[6], buf[7], buf[8]);
    w.tri(a, b, c);
    f.seekg(2, std::ios::cur);  // attribute byte count
  }
  return w.build();
}

Mesh loadAsciiSTL(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw IOError("Cannot open: " + path);
  Welder w;
  std::string tok;
  std::array<int, 3> corner;
  int nc = 0;
  while (f >> tok) {
    if (tok == "vertex") {
      float x, y, z;
      f >> x >> y >> z;
      corner[nc++] = w.add(x, y, z);
      if (nc == 3) {
        w.tri(corner[0], corner[1], corner[2]);
        nc = 0;
      }
    }
  }
  return w.build();
}
}  // namespace

Mesh loadSTL(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw IOError("Cannot open: " + path);
  f.seekg(0, std::ios::end);
  const std::streamoff size = f.tellg();
  f.seekg(0, std::ios::beg);
  if (size < 84) return loadAsciiSTL(path);

  char header[80];
  f.read(header, 80);
  uint32_t ntri = 0;
  f.read(reinterpret_cast<char*>(&ntri), 4);
  // Binary STL is exactly 84 + 50*ntri bytes; anything else is ASCII.
  if (static_cast<std::streamoff>(84 + 50ull * ntri) == size)
    return loadBinarySTL(f, ntri);
  return loadAsciiSTL(path);
}

}  // namespace ma::io
