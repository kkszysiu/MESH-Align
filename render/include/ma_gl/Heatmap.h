#pragma once
#include "ma_gl/GL.h"
#include <Eigen/Core>

namespace ma::gl {

// Tolerance-band colour for a signed deviation value (mm):
//   |v| <= inTol  -> green, |v| <= warn -> yellow, else red (+) / blue (-).
Eigen::Vector4f bandColor(float signedVal, float inTol, float warn);

// Create / refresh a 256x1 RGBA8 LUT texture spanning [-range, range] using the
// band colouring. The shader samples it by t = (v - (-range)) / (2*range).
GLuint makeBandLutTexture();
void updateBandLutTexture(GLuint tex, float inTol, float warn, float range);

}  // namespace ma::gl
