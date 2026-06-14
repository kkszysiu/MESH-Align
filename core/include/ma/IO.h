#pragma once
#include "ma/Mesh.h"

#include <Eigen/Core>
#include <stdexcept>
#include <string>

namespace ma::io {

// Thrown on any load/save failure with a human-readable message.
struct IOError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// Load a mesh by file extension: .stl (ascii/binary), .obj, .ply.
// Throws IOError on failure. Vertices are welded where the format is non-indexed.
Mesh loadMesh(const std::string& path);

// Save a mesh by extension: .stl (binary), .obj, .ply (ascii). Throws IOError.
void saveMesh(const std::string& path, const Mesh& mesh);

// Write a 4x4 transform (row-major) as a small text file.
void saveTransform(const std::string& path, const Eigen::Matrix4d& T);

// Return a copy of `mesh` with positions and normals transformed by T.
Mesh transformed(const Mesh& mesh, const Eigen::Matrix4d& T);

// Lower-cased extension without the dot ("" if none).
std::string extensionOf(const std::string& path);

// Format-specific loaders (used by loadMesh; exposed for testing).
Mesh loadSTL(const std::string& path);
Mesh loadOBJ(const std::string& path);
Mesh loadPLY(const std::string& path);

// Binary PLY writer (lives in the tinyply TU).
void savePLYBinary(const std::string& path, const Mesh& mesh);

}  // namespace ma::io
