#pragma once
#include <Eigen/Core>

namespace ma::gl {

// Orbit (turntable) camera around a target point, with standard CAD view presets.
// World is Z-up: Top looks down -Z, Front looks along +Y, Right looks along -X.
class Camera {
 public:
  enum class Preset { Top, Bottom, Front, Back, Right, Left, Iso };

  void setViewportSize(int w, int h);
  void fit(const Eigen::Vector3f& bmin, const Eigen::Vector3f& bmax);

  void orbit(float dxPixels, float dyPixels);   // left-drag
  void pan(float dxPixels, float dyPixels);      // middle/right-drag
  void dolly(float scrollSteps);                 // wheel

  void setPreset(Preset p);

  Eigen::Matrix4f view() const;
  Eigen::Matrix4f proj() const;
  Eigen::Vector3f eye() const;
  const Eigen::Vector3f& target() const { return target_; }

 private:
  Eigen::Vector3f target_ = Eigen::Vector3f::Zero();
  float distance_ = 5.0f;
  float azimuthDeg_ = -45.0f;     // around +Z
  float elevationDeg_ = 30.0f;    // from XY plane
  float fovYDeg_ = 40.0f;
  float sceneRadius_ = 1.0f;
  int vpW_ = 1, vpH_ = 1;
};

}  // namespace ma::gl
