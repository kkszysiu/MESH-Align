#pragma once
#include "ma/Mesh.h"
#include "ma/Types.h"

namespace ma {

// Inspect a mesh for issues relevant to scan-to-CAD alignment.
// Pure analysis — never modifies the mesh. Tolerant of holes / non-manifold input.
ValidationReport validate(const Mesh& mesh);

}  // namespace ma
