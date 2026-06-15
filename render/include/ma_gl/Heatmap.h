#pragma once
#include "ma_gl/GL.h"
#include <Eigen/Core>

namespace ma::gl {

// Colour for a signed deviation value (mm): green inside the tolerance band
// (|v| <= inTol), otherwise a diverging blue (under) -> white (zero) -> red
// (over) ramp across [-range, range].
Eigen::Vector4f bandColor(float signedVal, float inTol, float range);

// Create / refresh a 256x1 RGBA8 LUT texture spanning [-range, range].
// The shader samples it by t = (v - (-range)) / (2*range).
GLuint makeBandLutTexture();
void updateBandLutTexture(GLuint tex, float inTol, float range);

}  // namespace ma::gl
