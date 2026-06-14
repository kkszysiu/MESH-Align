#pragma once
#include "ma/Mesh.h"

namespace ma {

// Axis-aligned box centred at the origin, with per-vertex (flat) normals.
Mesh makeBox(double sx, double sy, double sz);

// A small stand-in "part" used for M0 viewport bring-up: a flat base slab with
// two raised bosses, deliberately asymmetric so orientation is visually obvious.
// Replaced by real scan import in M1.
Mesh makeDemoPart();

// Capped cylinder along +Z, centred at the origin. For cylinder/orientation tests.
Mesh makeCylinder(double radius, double height, int segments = 48);

}  // namespace ma
