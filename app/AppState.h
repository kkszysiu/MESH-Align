#pragma once
#include "PipelineRunner.h"
#include "ma/IO.h"
#include "ma/CylinderExtractor.h"
#include "ma/Geometry.h"
#include "ma/Mesh.h"
#include "ma/MeshValidation.h"
#include "ma/Orienter.h"
#include "ma/Picking.h"
#include "ma/Types.h"

#include <Eigen/Geometry>
#include <vector>
#include "ma_gl/Camera.h"
#include "ma_gl/Framebuffer.h"
#include "ma_gl/MeshRenderer.h"
#include "ma_gl/PlaneRenderer.h"
#include "ma_gl/TriadRenderer.h"

#include <cstdio>
#include <deque>
#include <exception>
#include <string>

namespace ma::ui {

// Single source of truth shared across the panels (immediate-mode UI).
struct AppState {
  // --- data ---
  ma::Mesh mesh;
  ma::AnalysisResult result;      // populated by the pipeline in M2+
  bool hasResult = false;
  ma::ValidationReport validation;
  bool hasValidation = false;

  // --- manual override of the orientation ---
  Eigen::Matrix4d userTransform = Eigen::Matrix4d::Identity();
  bool userEdited = false;

  // --- rendering ---
  ma::gl::Camera camera;
  ma::gl::Framebuffer fbo;
  ma::gl::MeshRenderer meshRenderer;
  ma::gl::TriadRenderer triad;
  ma::gl::PlaneRenderer planeRenderer;

  // --- analysis ---
  PipelineRunner pipeline;
  bool showDatums = true;

  // --- gizmo ---
  bool showGizmo = false;
  int gizmoOp = 0;  // 0 = rotate, 1 = translate

  // --- interactive picking ---
  enum class PickMode { None, ThreePoint, Hole };
  PickMode pickMode = PickMode::None;
  ma::DatumRole pendingRole = ma::DatumRole::Top;
  std::vector<Eigen::Vector3d> pickedPoints;  // part space

  // cached topology for hole picking (rebuilt on load)
  bool topoValid = false;
  ma::FaceData faceData;
  std::vector<std::array<int, 3>> adjacency;
  void ensureTopology() {
    if (topoValid || mesh.empty()) return;
    faceData = ma::computeFaceData(mesh);
    adjacency = ma::triangleAdjacency(mesh);
    topoValid = true;
  }

  // --- view options ---
  Eigen::Vector3f bgColor{0.16f, 0.16f, 0.17f};
  Eigen::Vector3f partColor{0.62f, 0.62f, 0.64f};
  bool showTriad = true;
  bool showGrid = false;

  // --- pipeline status (placeholder until M2) ---
  ma::Stage stage = ma::Stage::Idle;
  float progress = 0.0f;

  // --- log ---
  std::deque<std::string> log;
  void logLine(const std::string& s) {
    log.push_back(s);
    while (log.size() > 200) log.pop_front();
  }

  // Current part->datum transform (identity until analysis/manual edits).
  Eigen::Matrix4d currentTransform() const {
    if (userEdited) return userTransform;
    return hasResult ? result.orientation.transform : Eigen::Matrix4d::Identity();
  }
  Eigen::Matrix4f modelF() const { return currentTransform().cast<float>(); }

  // Decompose the current transform into part-space datum axes + origin.
  void currentFrame(Eigen::Vector3d& X, Eigen::Vector3d& Y, Eigen::Vector3d& Z,
                    Eigen::Vector3d& origin) const {
    const Eigen::Matrix4d T = currentTransform();
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    X = R.row(0);
    Y = R.row(1);
    Z = R.row(2);
    origin = -R.transpose() * T.topRightCorner<3, 1>();
  }

  // Assign a part-space normal to a datum role and rebuild the frame.
  void assignDatum(ma::DatumRole role, const Eigen::Vector3d& normal) {
    Eigen::Vector3d X, Y, Z, origin;
    currentFrame(X, Y, Z, origin);
    Eigen::Vector3d Zh = Z, Xh = X;
    std::string which = "?";
    if (role == ma::DatumRole::Top) { Zh = normal; Xh = X; which = "Top"; }
    else if (role == ma::DatumRole::Right) { Zh = Z; Xh = normal; which = "Right"; }
    else if (role == ma::DatumRole::Front) { Zh = Z; Xh = normal.cross(Z); which = "Front"; }
    auto r = ma::orientFromAxes(mesh, Zh, Xh, origin, "manual: " + which + " datum", 1.0);
    userTransform = r.transform;
    userEdited = true;
    logLine("Assigned " + which + " datum");
    frameAligned();
  }

  // Compose a world-space rotation onto the current datum frame (nudge).
  void applyWorldRotation(const Eigen::Vector3d& axis, double degrees) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(degrees * 3.14159265358979323846 / 180.0, axis.normalized())
            .toRotationMatrix();
    userTransform = M * currentTransform();
    userEdited = true;
  }

  void setUserTransform(const Eigen::Matrix4d& T) {
    userTransform = T;
    userEdited = true;
  }

  void startPick(PickMode mode, ma::DatumRole role) {
    if (mesh.empty()) { logLine("Load a mesh first"); return; }
    pickMode = mode;
    pendingRole = role;
    pickedPoints.clear();
    logLine(mode == PickMode::ThreePoint ? "Pick 3 points on the surface"
                                         : "Click a hole/bore to fit a cylinder");
  }
  void cancelPick() { pickMode = PickMode::None; pickedPoints.clear(); }

  // Handle a pick ray expressed in PART space.
  void pickRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir) {
    if (pickMode == PickMode::None) return;
    const ma::RayHit hit = ma::raycast(mesh, origin, dir);
    if (!hit.hit) { logLine("Pick missed the mesh"); return; }

    if (pickMode == PickMode::ThreePoint) {
      pickedPoints.push_back(hit.point);
      logLine("  point " + std::to_string(pickedPoints.size()) + "/3");
      if (pickedPoints.size() == 3) {
        Eigen::Vector3d n =
            (pickedPoints[1] - pickedPoints[0]).cross(pickedPoints[2] - pickedPoints[0]);
        if (n.norm() < 1e-12) { logLine("Degenerate 3-point plane"); cancelPick(); return; }
        n.normalize();
        if (n.dot(origin - pickedPoints[0]) < 0) n = -n;  // face the camera
        assignDatum(pendingRole, n);
        cancelPick();
      }
    } else if (pickMode == PickMode::Hole) {
      ensureTopology();
      auto region = ma::growSmoothRegion(mesh, faceData, adjacency, hit.face, 25.0);
      std::vector<char> skip(static_cast<size_t>(mesh.F.rows()), 1);
      for (int f : region) skip[static_cast<size_t>(f)] = 0;
      ma::CylinderExtractionParams cp;
      cp.ransacIterations = 400;
      cp.minInliers = 8;
      auto cyls = ma::extractCylinders(mesh, faceData, skip, cp);
      if (cyls.empty()) { logLine("No cylinder found at that spot"); cancelPick(); return; }
      char buf[96];
      std::snprintf(buf, sizeof(buf), "Fit cylinder r=%.2f from %zu faces", cyls[0].radius,
                    region.size());
      logLine(buf);
      assignDatum(pendingRole, cyls[0].axisDir);
      cancelPick();
    }
  }

  void runAnalysis() {
    if (mesh.empty()) {
      logLine("Load a mesh first");
      return;
    }
    if (pipeline.running()) return;
    logLine("Running analysis...");
    pipeline.start(mesh);
  }

  void applyResult(ma::AnalysisResult r) {
    result = std::move(r);
    hasResult = true;
    userEdited = false;  // start from the fresh auto result
    const auto& o = result.orientation;
    logLine("Aligned: " + o.method);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "  rot XYZ = %.1f, %.1f, %.1f deg   confidence %.0f%%",
                  o.eulerXYZ_deg.x(), o.eulerXYZ_deg.y(), o.eulerXYZ_deg.z(),
                  o.confidence * 100.0);
    logLine(buf);
    logLine("  " + std::to_string(result.planes.size()) + " planes, " +
            std::to_string(result.cylinders.size()) + " bores");
    frameAligned();
  }

  void revertToAuto() {
    if (hasResult) {
      userEdited = false;
      logLine("Reverted to auto orientation");
      frameAligned();
    }
  }

  // Frame the part in its current (possibly aligned) pose.
  void frameAligned() {
    if (mesh.empty()) return;
    const Eigen::Matrix4d T = currentTransform();
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    const Eigen::Vector3d t = T.topRightCorner<3, 1>();
    const Eigen::Vector3d mn = mesh.bboxMin(), mx = mesh.bboxMax();
    Eigen::Vector3f amn(1e30f, 1e30f, 1e30f), amx(-1e30f, -1e30f, -1e30f);
    for (int i = 0; i < 8; ++i) {
      const Eigen::Vector3d c((i & 1) ? mx.x() : mn.x(), (i & 2) ? mx.y() : mn.y(),
                              (i & 4) ? mx.z() : mn.z());
      const Eigen::Vector3f p = (R * c + t).cast<float>();
      amn = amn.cwiseMin(p);
      amx = amx.cwiseMax(p);
    }
    camera.fit(amn, amx);
  }

  // Load a mesh file, upload it, validate, and frame it. Logs on failure.
  bool loadMeshFile(const std::string& path) {
    try {
      mesh = ma::io::loadMesh(path);
    } catch (const std::exception& e) {
      logLine(std::string("Load failed: ") + e.what());
      return false;
    }
    meshRenderer.upload(mesh);
    validation = ma::validate(mesh);
    hasValidation = true;
    result = ma::AnalysisResult{};
    hasResult = false;  // back to identity pose
    userEdited = false;
    topoValid = false;  // rebuild adjacency for the new mesh on demand
    cancelPick();
    frameMesh();
    logLine("Loaded " + mesh.name + "  (" + std::to_string(mesh.numFaces()) +
            " tris, " + std::to_string(mesh.numVertices()) + " verts)");
    for (const auto& n : validation.notes) logLine("  - " + n);
    return true;
  }

  bool exportAlignedMesh(const std::string& path) {
    try {
      ma::io::saveMesh(path, ma::io::transformed(mesh, currentTransform()));
    } catch (const std::exception& e) {
      logLine(std::string("Export failed: ") + e.what());
      return false;
    }
    logLine("Exported aligned mesh -> " + path);
    return true;
  }

  bool exportTransform(const std::string& path) {
    try {
      ma::io::saveTransform(path, currentTransform());
    } catch (const std::exception& e) {
      logLine(std::string("Export failed: ") + e.what());
      return false;
    }
    logLine("Exported transform -> " + path);
    return true;
  }

  // Frame the current mesh in the viewport.
  void frameMesh() {
    if (mesh.empty()) {
      camera.fit(Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(1, 1, 1));
      return;
    }
    camera.fit(mesh.bboxMin().cast<float>(), mesh.bboxMax().cast<float>());
  }
};

}  // namespace ma::ui
