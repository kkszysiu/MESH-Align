#include "ma_gl/Camera.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace ma::gl {

namespace {
constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;

Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center,
                       const Eigen::Vector3f& upHint) {
  Eigen::Vector3f f = (center - eye).normalized();
  Eigen::Vector3f up = upHint;
  // Guard against up parallel to view direction (Top/Bottom presets).
  if (std::abs(f.dot(up.normalized())) > 0.999f) up = Eigen::Vector3f(0, 1, 0);
  Eigen::Vector3f s = f.cross(up).normalized();
  Eigen::Vector3f u = s.cross(f);
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.row(0).head<3>() = s;
  m.row(1).head<3>() = u;
  m.row(2).head<3>() = -f;
  m(0, 3) = -s.dot(eye);
  m(1, 3) = -u.dot(eye);
  m(2, 3) = f.dot(eye);
  return m;
}

Eigen::Matrix4f perspective(float fovYRad, float aspect, float zn, float zf) {
  Eigen::Matrix4f m = Eigen::Matrix4f::Zero();
  const float t = std::tan(fovYRad * 0.5f);
  m(0, 0) = 1.0f / (aspect * t);
  m(1, 1) = 1.0f / t;
  m(2, 2) = (zf + zn) / (zn - zf);
  m(2, 3) = (2.0f * zf * zn) / (zn - zf);
  m(3, 2) = -1.0f;
  return m;
}
}  // namespace

void Camera::setViewportSize(int w, int h) {
  vpW_ = std::max(1, w);
  vpH_ = std::max(1, h);
}

void Camera::fit(const Eigen::Vector3f& bmin, const Eigen::Vector3f& bmax) {
  target_ = 0.5f * (bmin + bmax);
  sceneRadius_ = std::max(0.001f, 0.5f * (bmax - bmin).norm());
  distance_ = sceneRadius_ / std::sin(0.5f * fovYDeg_ * kDeg2Rad) * 1.15f;
}

void Camera::orbit(float dxPixels, float dyPixels) {
  azimuthDeg_ -= dxPixels * 0.4f;
  elevationDeg_ = std::clamp(elevationDeg_ + dyPixels * 0.4f, -89.0f, 89.0f);
}

void Camera::pan(float dxPixels, float dyPixels) {
  // Move target in the camera's screen plane, scaled by distance.
  Eigen::Matrix4f v = view();
  Eigen::Vector3f right = v.row(0).head<3>();
  Eigen::Vector3f up = v.row(1).head<3>();
  const float scale = distance_ * std::tan(0.5f * fovYDeg_ * kDeg2Rad) * 2.0f / vpH_;
  target_ += (-dxPixels * right + dyPixels * up) * scale;
}

void Camera::dolly(float scrollSteps) {
  distance_ *= std::pow(0.9f, scrollSteps);
  distance_ = std::clamp(distance_, sceneRadius_ * 0.05f, sceneRadius_ * 50.0f);
}

void Camera::setPreset(Preset p) {
  switch (p) {
    case Preset::Top:    azimuthDeg_ = -90.0f; elevationDeg_ = 89.999f;  break;
    case Preset::Bottom: azimuthDeg_ = -90.0f; elevationDeg_ = -89.999f; break;
    case Preset::Front:  azimuthDeg_ = -90.0f; elevationDeg_ = 0.0f;     break;
    case Preset::Back:   azimuthDeg_ = 90.0f;  elevationDeg_ = 0.0f;     break;
    case Preset::Right:  azimuthDeg_ = 0.0f;   elevationDeg_ = 0.0f;     break;
    case Preset::Left:   azimuthDeg_ = 180.0f; elevationDeg_ = 0.0f;     break;
    case Preset::Iso:    azimuthDeg_ = -45.0f; elevationDeg_ = 35.264f;  break;
  }
}

Eigen::Vector3f Camera::eye() const {
  const float az = azimuthDeg_ * kDeg2Rad;
  const float el = elevationDeg_ * kDeg2Rad;
  Eigen::Vector3f dir(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                      std::sin(el));
  return target_ + distance_ * dir;
}

Eigen::Matrix4f Camera::view() const {
  return lookAt(eye(), target_, Eigen::Vector3f(0, 0, 1));
}

Eigen::Matrix4f Camera::proj() const {
  const float aspect = static_cast<float>(vpW_) / static_cast<float>(vpH_);
  const float zn = std::max(0.01f, distance_ - sceneRadius_ * 4.0f);
  const float zf = distance_ + sceneRadius_ * 8.0f;
  return perspective(fovYDeg_ * kDeg2Rad, aspect, zn, zf);
}

}  // namespace ma::gl
