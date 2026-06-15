#pragma once
#include "PipelineRunner.h"
#include "ma/IO.h"
#include "ma/Bvh.h"
#include "ma/CylinderExtractor.h"
#include "ma/Deviation.h"
#include "ma/Geometry.h"
#include "ma/Mesh.h"
#include "ma/MeshValidation.h"
#include "ma/Orienter.h"
#include "ma/Picking.h"
#include "ma/Types.h"
#include "ma_gl/Heatmap.h"

#include <portable-file-dialogs.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <functional>
#include <memory>
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

  // --- per-datum adjustment (live preview, committed on Apply) ---
  int editRow = -1;          // expanded detected-plane row (-1 = none)
  int adjAxis = 2;           // 0=X 1=Y 2=Z datum axis to rotate about / offset along
  float adjAngleDeg = 0.0f;  // pending rotation about the datum axis (through origin)
  float adjOffset = 0.0f;    // pending translation along the datum axis (mm)
  std::vector<ma::PlaneResidual> planeResiduals;  // per detected-plane (lazy)
  bool residualsValid = false;

  // --- rendering ---
  ma::gl::Camera camera;
  ma::gl::Framebuffer fbo;
  ma::gl::MeshRenderer meshRenderer;
  ma::gl::TriadRenderer triad;
  ma::gl::PlaneRenderer planeRenderer;

  // --- analysis ---
  PipelineRunner pipeline;
  bool showDatums = true;

  // --- app lifecycle ---
  bool wantQuit = false;

  // --- gizmo ---
  bool showGizmo = false;
  int gizmoOp = 0;  // 0 = rotate, 1 = translate

  // --- deviation / heatmap ---
  ma::Mesh reference;
  bool hasReference = false;
  ma::Bvh refBvh;
  ma::DeviationField deviation;
  bool hasDeviation = false;
  bool showHeatmap = false;
  int devMode = 0;  // 0 = vs reference (ICP), 1 = vs Top datum plane
  ma::ToleranceBands bands;
  Eigen::Matrix4d scanToRef = Eigen::Matrix4d::Identity();
  float lutRange = 1.0f;
  bool autoRange = true;
  float autoRangeValue = 1.0f;  // data-driven range from the last run
  float manualRange = 1.0f;     // used when autoRange is off
  GLuint lutTex = 0;  // owned; created in main once GL is up

  void applyRange() {
    lutRange = autoRange ? autoRangeValue : std::max(manualRange, 1e-3f);
    refreshLut();
  }

  struct DevStats { int inTol = 0, warn = 0, out = 0; double pctIn = 0, rms = 0; };
  DevStats deviationStats() const {
    DevStats s;
    double sse = 0;
    for (Eigen::Index i = 0; i < deviation.signedDistMm.size(); ++i) {
      const double d = deviation.signedDistMm(i);
      const double a = std::abs(d);
      sse += d * d;
      if (a <= bands.inTolMm) ++s.inTol;
      else if (a <= bands.warnMm) ++s.warn;
      else ++s.out;
    }
    const int n = static_cast<int>(deviation.signedDistMm.size());
    s.pctIn = n ? 100.0 * s.inTol / n : 0.0;
    s.rms = n ? std::sqrt(sse / n) : 0.0;
    return s;
  }

  void refreshLut() {
    if (lutTex) ma::gl::updateBandLutTexture(lutTex, (float)bands.inTolMm, lutRange);
  }

  bool loadReference(const std::string& path) {
    try {
      reference = ma::io::loadMesh(path);
    } catch (const std::exception& e) {
      logLine(std::string("Reference load failed: ") + e.what());
      return false;
    }
    refBvh.build(reference);
    hasReference = true;
    logLine("Reference: " + reference.name + " (" + std::to_string(reference.numFaces()) + " tris)");
    return true;
  }

  void runDeviation() {
    if (mesh.empty()) { logLine("Load a mesh first"); return; }
    if (devMode == 0) {
      if (!hasReference) { logLine("Open a reference mesh first"); return; }
      logLine("Best-fitting scan to reference (ICP)...");
      ma::IcpResult icp = ma::icpToReference(mesh, reference, refBvh);
      scanToRef = icp.transform;
      char b[96];
      std::snprintf(b, sizeof(b), "ICP rms = %.3f mm (%d iters)", icp.rms, icp.iterations);
      logLine(b);
      deviation = ma::deviationToReference(mesh, reference, refBvh, scanToRef, bands);
    } else {
      // vs Top datum plane (fallback to symmetry plane)
      Eigen::Vector3d pt = mesh.centroid(), n(0, 0, 1);
      bool found = false;
      for (const auto& p : result.planes)
        if (p.role == ma::DatumRole::Top) { pt = p.point; n = p.normal; found = true; break; }
      if (!found && !result.symmetry.empty()) { pt = result.symmetry[0].point; n = result.symmetry[0].normal; }
      deviation = ma::deviationToPlane(mesh, pt, n, bands);
    }
    hasDeviation = true;
    showHeatmap = true;

    Eigen::VectorXf sc = deviation.signedDistMm.cast<float>();
    meshRenderer.uploadScalars(sc);
    autoRangeValue = std::max({(float)std::abs(deviation.minMm), (float)std::abs(deviation.maxMm),
                               (float)bands.inTolMm * 2.0f, 1e-3f});
    applyRange();

    DevStats st = deviationStats();
    char b[160];
    std::snprintf(b, sizeof(b), "Deviation: %.3f..%.3f mm, RMS %.3f, %.0f%% in tolerance",
                  deviation.minMm, deviation.maxMm, st.rms, st.pctIn);
    logLine(b);
  }

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

  // Committed part->datum transform (identity until analysis/manual edits).
  Eigen::Matrix4d currentTransform() const {
    if (userEdited) return userTransform;
    return hasResult ? result.orientation.transform : Eigen::Matrix4d::Identity();
  }

  // Pending (un-applied) adjustment about a datum axis through the origin.
  Eigen::Matrix4d pendingDelta() const {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    if (adjAngleDeg == 0.0f && adjOffset == 0.0f) return M;
    Eigen::Vector3d e = Eigen::Vector3d::Unit(adjAxis);
    M.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(adjAngleDeg * 3.14159265358979323846 / 180.0, e).toRotationMatrix();
    M.topRightCorner<3, 1>() = e * adjOffset;
    return M;
  }
  // What the viewport shows / picking uses: committed pose + live preview.
  Eigen::Matrix4d previewTransform() const { return pendingDelta() * currentTransform(); }
  Eigen::Matrix4f modelF() const { return previewTransform().cast<float>(); }

  void clearPending() { adjAngleDeg = 0.0f; adjOffset = 0.0f; }

  void applyAdjustment() {
    if (adjAngleDeg == 0.0f && adjOffset == 0.0f) return;
    userTransform = previewTransform();
    userEdited = true;
    clearPending();
    logLine("Applied datum adjustment");
    frameAligned();
  }
  void resetAdjustment() { clearPending(); }

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
    clearPending();
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
    clearPending();
  }

  void setUserTransform(const Eigen::Matrix4d& T) {
    userTransform = T;
    userEdited = true;
    clearPending();
  }

  // Lazily compute per-detected-plane flatness residuals (for the datum editor).
  const ma::PlaneResidual& residualFor(size_t i) {
    if (!residualsValid) {
      planeResiduals.clear();
      planeResiduals.reserve(result.planes.size());
      for (const auto& p : result.planes)
        planeResiduals.push_back(ma::planeResidual(mesh, p.point, p.normal, p.inlierFaces));
      residualsValid = true;
    }
    static ma::PlaneResidual zero;
    return i < planeResiduals.size() ? planeResiduals[i] : zero;
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
      result.cylinders.push_back(cyls[0]);  // surface it in the Detected list
      char buf[96];
      std::snprintf(buf, sizeof(buf), "Fit cylinder dia %.2f L %.2f from %zu faces",
                    2.0 * cyls[0].radius, cyls[0].length, region.size());
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
    clearPending();
    residualsValid = false;
    editRow = -1;
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
      clearPending();
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
    hasDeviation = false;
    showHeatmap = false;
    clearPending();
    residualsValid = false;
    editRow = -1;
    cancelPick();
    frameMesh();
    logLine("Loaded " + mesh.name + "  (" + std::to_string(mesh.numFaces()) +
            " tris, " + std::to_string(mesh.numVertices()) + " verts)");
    for (const auto& n : validation.notes) logLine("  - " + n);
    return true;
  }

  bool exportAlignedMesh(const std::string& pathIn) {
    // The native save panel may not append an extension; default to .stl.
    std::string path = pathIn;
    const std::string ext = ma::io::extensionOf(path);
    if (ext != "stl" && ext != "obj" && ext != "ply") path += ".stl";
    try {
      ma::io::saveMesh(path, ma::io::transformed(mesh, currentTransform()));
    } catch (const std::exception& e) {
      logLine(std::string("Export failed: ") + e.what());
      return false;
    }
    logLine("Exported aligned mesh -> " + path);
    return true;
  }

  bool exportTransform(const std::string& pathIn) {
    std::string path = pathIn;
    if (ma::io::extensionOf(path) != "txt") path += ".txt";
    try {
      ma::io::saveTransform(path, currentTransform());
    } catch (const std::exception& e) {
      logLine(std::string("Export failed: ") + e.what());
      return false;
    }
    logLine("Exported transform -> " + path);
    return true;
  }

  // ---- non-blocking native file dialogs ----
  // Blocking pfd calls freeze the render loop and the dialog ends up stuck
  // behind the (frozen, on-top) GL window. Instead we kick off the dialog and
  // poll it each frame, running the callback when the user responds.
  std::unique_ptr<pfd::open_file> openDlg_;
  std::unique_ptr<pfd::save_file> saveDlg_;
  std::function<void(const std::string&)> onPath_;

  bool dialogOpen() const { return openDlg_ || saveDlg_; }

  void beginOpen(const std::string& title, const std::vector<std::string>& filters,
                 std::function<void(const std::string&)> cb) {
    if (dialogOpen()) return;
    // NOTE: pfd's default-path arg is a *folder*; passing a filename (or ".")
    // makes osascript error out and no dialog appears. Empty -> platform default.
    openDlg_ = std::make_unique<pfd::open_file>(title, std::string(), filters);
    onPath_ = std::move(cb);
  }
  void beginSave(const std::string& title, const std::vector<std::string>& filters,
                 std::function<void(const std::string&)> cb) {
    if (dialogOpen()) return;
    saveDlg_ = std::make_unique<pfd::save_file>(title, std::string(), filters);
    onPath_ = std::move(cb);
  }
  void pollDialogs() {
    if (openDlg_ && openDlg_->ready(0)) {
      const auto res = openDlg_->result();
      openDlg_.reset();
      if (!res.empty() && onPath_) onPath_(res[0]);
      onPath_ = nullptr;
    }
    if (saveDlg_ && saveDlg_->ready(0)) {
      const auto res = saveDlg_->result();
      saveDlg_.reset();
      if (!res.empty() && onPath_) onPath_(res);
      onPath_ = nullptr;
    }
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
