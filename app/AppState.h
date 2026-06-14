#pragma once
#include "PipelineRunner.h"
#include "ma/IO.h"
#include "ma/Mesh.h"
#include "ma/MeshValidation.h"
#include "ma/Types.h"
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

  // --- rendering ---
  ma::gl::Camera camera;
  ma::gl::Framebuffer fbo;
  ma::gl::MeshRenderer meshRenderer;
  ma::gl::TriadRenderer triad;
  ma::gl::PlaneRenderer planeRenderer;

  // --- analysis ---
  PipelineRunner pipeline;
  bool showDatums = true;

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
    return hasResult ? result.orientation.transform : Eigen::Matrix4d::Identity();
  }
  Eigen::Matrix4f modelF() const { return currentTransform().cast<float>(); }

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
