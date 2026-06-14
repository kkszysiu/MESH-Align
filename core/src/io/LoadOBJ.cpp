#include "ma/IO.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <vector>

namespace ma::io {

Mesh loadOBJ(const std::string& path) {
  tinyobj::ObjReaderConfig cfg;
  cfg.triangulate = true;
  cfg.vertex_color = false;

  tinyobj::ObjReader reader;
  if (!reader.ParseFromFile(path, cfg)) {
    throw IOError("OBJ parse failed: " +
                  (reader.Error().empty() ? path : reader.Error()));
  }

  const tinyobj::attrib_t& attrib = reader.GetAttrib();
  const auto& shapes = reader.GetShapes();

  Mesh m;
  const size_t nv = attrib.vertices.size() / 3;
  m.V.resize(static_cast<Eigen::Index>(nv), 3);
  for (size_t i = 0; i < nv; ++i)
    m.V.row(static_cast<Eigen::Index>(i)) << attrib.vertices[3 * i + 0],
        attrib.vertices[3 * i + 1], attrib.vertices[3 * i + 2];

  std::vector<Eigen::Vector3i> faces;
  for (const auto& shape : shapes) {
    size_t offset = 0;
    for (size_t fi = 0; fi < shape.mesh.num_face_vertices.size(); ++fi) {
      const int fv = shape.mesh.num_face_vertices[fi];
      if (fv == 3) {
        faces.emplace_back(shape.mesh.indices[offset + 0].vertex_index,
                           shape.mesh.indices[offset + 1].vertex_index,
                           shape.mesh.indices[offset + 2].vertex_index);
      }
      offset += static_cast<size_t>(fv);
    }
  }
  m.F.resize(static_cast<Eigen::Index>(faces.size()), 3);
  for (size_t i = 0; i < faces.size(); ++i)
    m.F.row(static_cast<Eigen::Index>(i)) = faces[i].transpose();
  return m;
}

}  // namespace ma::io
