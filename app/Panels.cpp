#include "Panels.h"
#include "AppState.h"

#include "ma_gl/GL.h"
#include <imgui.h>
#include <ImGuizmo.h>
#include <portable-file-dialogs.h>

#include <cstdio>

namespace ma::ui {

using ma::gl::Camera;

namespace {
void openMeshDialog(AppState& s) {
  auto sel = pfd::open_file("Open mesh", ".",
                            {"Meshes (.stl .obj .ply)", "*.stl *.obj *.ply",
                             "All files", "*"})
                 .result();
  if (!sel.empty()) s.loadMeshFile(sel[0]);
}

void exportMeshDialog(AppState& s) {
  if (s.mesh.empty()) {
    s.logLine("Nothing to export");
    return;
  }
  auto path = pfd::save_file("Export aligned mesh", "aligned.stl",
                             {"STL", "*.stl", "OBJ", "*.obj", "PLY", "*.ply"})
                  .result();
  if (!path.empty()) s.exportAlignedMesh(path);
}

void exportTransformDialog(AppState& s) {
  auto path = pfd::save_file("Export transform", "transform.txt", {"Text", "*.txt"}).result();
  if (!path.empty()) s.exportTransform(path);
}
}  // namespace

void drawMenuBar(AppState& s) {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Open Mesh...", "Ctrl+O")) openMeshDialog(s);
      if (ImGui::MenuItem("Export Aligned Mesh...", "Ctrl+E", false, !s.mesh.empty()))
        exportMeshDialog(s);
      if (ImGui::MenuItem("Export Transform...", nullptr, false, !s.mesh.empty()))
        exportTransformDialog(s);
      ImGui::Separator();
      if (ImGui::MenuItem("Quit", "Ctrl+Q")) s.logLine("Quit requested");
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit")) {
      if (ImGui::MenuItem("Revert to Auto", nullptr, false, s.hasResult)) s.revertToAuto();
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Analyze")) {
      if (ImGui::MenuItem("Run Analysis", "F5", false, !s.mesh.empty() && !s.pipeline.running()))
        s.runAnalysis();
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
      ImGui::MenuItem("Show Triad", nullptr, &s.showTriad);
      ImGui::MenuItem("Show Datum Planes", nullptr, &s.showDatums);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
}

void drawLeftPanel(AppState& s) {
  ImGui::Begin("Inspector");

  if (ImGui::CollapsingHeader("File", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextWrapped("%s", s.mesh.name.empty() ? "(none)" : s.mesh.name.c_str());
    ImGui::TextDisabled("units: %s", s.mesh.units.c_str());
    if (ImGui::Button("Open Mesh...", ImVec2(-FLT_MIN, 0))) openMeshDialog(s);
    ImGui::TextDisabled("or drag a .stl / .obj / .ply file here");
  }

  if (ImGui::CollapsingHeader("Mesh Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Triangles : %lld", (long long)s.mesh.numFaces());
    ImGui::Text("Vertices  : %lld", (long long)s.mesh.numVertices());
    ImGui::Text("Surface   : %.1f mm2", s.mesh.surfaceArea());
    ImGui::Text("Volume    : %.1f mm3", s.mesh.volume());
  }

  if (ImGui::CollapsingHeader("Validation", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (!s.hasValidation) {
      ImGui::TextDisabled("Load a mesh to validate");
    } else {
      const auto& v = s.validation;
      ImGui::Text("Components   : %d", v.components);
      ImGui::Text("Boundary edges : %d", v.boundaryEdges);
      ImGui::Text("Non-manifold  : %d", v.nonManifoldEdges);
      ImGui::Text("Degenerate    : %d", v.degenerateFaces);
      ImGui::Text("Duplicate vtx : %d", v.duplicateVertices);
      if (v.isWatertight)
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 0.4f, 1), "Watertight");
      else
        ImGui::TextColored(ImVec4(0.9f, 0.75f, 0.3f, 1), "Open / non-watertight");
    }
  }

  if (ImGui::CollapsingHeader("Analysis", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::BeginDisabled(s.mesh.empty() || s.pipeline.running());
    if (ImGui::Button(s.pipeline.running() ? "Analyzing..." : "Run Analysis",
                      ImVec2(-FLT_MIN, 0)))
      s.runAnalysis();
    ImGui::EndDisabled();
  }

  if (ImGui::CollapsingHeader("Pipeline", ImGuiTreeNodeFlags_DefaultOpen)) {
    const ma::Stage stages[] = {ma::Stage::Validation, ma::Stage::Normals,
                                ma::Stage::Planes, ma::Stage::Cylinders,
                                ma::Stage::Symmetry, ma::Stage::Orientation};
    if (s.pipeline.running()) {
      auto pr = s.pipeline.progress();
      ImGui::ProgressBar(pr.pct, ImVec2(-FLT_MIN, 0));
      ImGui::TextDisabled("%s - %s", ma::stageName(pr.stage), pr.msg.c_str());
    } else {
      for (ma::Stage st : stages)
        ImGui::TextDisabled("%s %s", s.hasResult ? "[x]" : "[ ]", ma::stageName(st));
    }
  }

  if (ImGui::CollapsingHeader("Orientation Result", ImGuiTreeNodeFlags_DefaultOpen)) {
    const auto& o = s.result.orientation;
    ImGui::Text("Rotation X : %+.2f deg", s.hasResult ? o.eulerXYZ_deg.x() : 0.0);
    ImGui::Text("Rotation Y : %+.2f deg", s.hasResult ? o.eulerXYZ_deg.y() : 0.0);
    ImGui::Text("Rotation Z : %+.2f deg", s.hasResult ? o.eulerXYZ_deg.z() : 0.0);
    ImGui::Text("Confidence : %.0f%%", s.hasResult ? o.confidence * 100.0 : 0.0);
    ImGui::Text("Method     : %s", s.hasResult ? o.method.c_str() : "-");
  }

  if (ImGui::CollapsingHeader("Log", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::BeginChild("log", ImVec2(0, 120), ImGuiChildFlags_Borders);
    for (const auto& line : s.log) ImGui::TextUnformatted(line.c_str());
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();
  }

  ImGui::End();
}

void drawRightPanel(AppState& s) {
  ImGui::Begin("Datums");

  if (ImGui::CollapsingHeader("View Preset", ImGuiTreeNodeFlags_DefaultOpen)) {
    const float w = (ImGui::GetContentRegionAvail().x - 2 * ImGui::GetStyle().ItemSpacing.x) / 3.0f;
    auto btn = [&](const char* label, Camera::Preset p) {
      if (ImGui::Button(label, ImVec2(w, 0))) s.camera.setPreset(p);
    };
    btn("Top", Camera::Preset::Top); ImGui::SameLine();
    btn("Front", Camera::Preset::Front); ImGui::SameLine();
    btn("Right", Camera::Preset::Right);
    btn("Iso", Camera::Preset::Iso); ImGui::SameLine();
    btn("Back", Camera::Preset::Back); ImGui::SameLine();
    btn("Left", Camera::Preset::Left);
    if (ImGui::Button("Frame Selection", ImVec2(-FLT_MIN, 0))) s.frameMesh();
  }

  if (ImGui::CollapsingHeader("Detected Planes", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (s.result.planes.empty()) {
      ImGui::TextDisabled("No analysis yet");
    } else {
      auto roleName = [](ma::DatumRole r) {
        switch (r) {
          case ma::DatumRole::Top: return "Top";
          case ma::DatumRole::Front: return "Front";
          case ma::DatumRole::Right: return "Right";
          default: return "-";
        }
      };
      for (size_t i = 0; i < s.result.planes.size(); ++i) {
        const auto& p = s.result.planes[i];
        ImGui::PushID(static_cast<int>(i));
        ImGui::Text("Plane %2zu  %3.0f%%  [%s]", i, p.confidence * 100.0, roleName(p.role));
        ImGui::SameLine();
        if (ImGui::SmallButton("T")) s.assignDatum(ma::DatumRole::Top, p.normal);
        ImGui::SameLine();
        if (ImGui::SmallButton("F")) s.assignDatum(ma::DatumRole::Front, p.normal);
        ImGui::SameLine();
        if (ImGui::SmallButton("R")) s.assignDatum(ma::DatumRole::Right, p.normal);
        ImGui::PopID();
      }
    }
    if (!s.result.cylinders.empty()) {
      ImGui::Separator();
      for (size_t i = 0; i < s.result.cylinders.size(); ++i)
        ImGui::Text("Bore %zu  r=%.2f", i, s.result.cylinders[i].radius);
    }
  }

  if (ImGui::CollapsingHeader("Actions", ImGuiTreeNodeFlags_DefaultOpen)) {
    const bool haveMesh = !s.mesh.empty();
    ImGui::BeginDisabled(!haveMesh);
    if (ImGui::Button("Export Aligned Mesh", ImVec2(-FLT_MIN, 0))) exportMeshDialog(s);
    if (ImGui::Button("Export Transform", ImVec2(-FLT_MIN, 0))) exportTransformDialog(s);
    ImGui::EndDisabled();

    ImGui::BeginDisabled(!s.hasResult);
    if (ImGui::Button("Revert to Auto", ImVec2(-FLT_MIN, 0))) s.revertToAuto();
    ImGui::EndDisabled();
    ImGui::Checkbox("Show datum planes", &s.showDatums);

    ImGui::Separator();
    static int roleIdx = 0;
    const char* roles[] = {"Top", "Front", "Right"};
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::Combo("##assignto", &roleIdx, roles, 3);
    auto role = [&]() {
      return roleIdx == 0 ? ma::DatumRole::Top
             : roleIdx == 1 ? ma::DatumRole::Front
                            : ma::DatumRole::Right;
    };
    ImGui::BeginDisabled(s.mesh.empty());
    if (ImGui::Button("Plane from 3 Points", ImVec2(-FLT_MIN, 0)))
      s.startPick(AppState::PickMode::ThreePoint, role());
    if (ImGui::Button("Pick Hole / Cylinder", ImVec2(-FLT_MIN, 0)))
      s.startPick(AppState::PickMode::Hole, role());
    ImGui::EndDisabled();
    if (s.pickMode != AppState::PickMode::None) {
      ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "Picking... (Esc to cancel)");
      if (s.pickMode == AppState::PickMode::ThreePoint)
        ImGui::Text("points: %zu/3", s.pickedPoints.size());
    }
  }

  if (ImGui::CollapsingHeader("Nudge Datum", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::BeginDisabled(!s.hasResult && !s.userEdited);
    ImGui::Checkbox("Gizmo", &s.showGizmo);
    if (s.showGizmo) {
      ImGui::SameLine();
      ImGui::RadioButton("Rotate", &s.gizmoOp, 0);
      ImGui::SameLine();
      ImGui::RadioButton("Move", &s.gizmoOp, 1);
    }
    static float step = 5.0f;
    ImGui::SetNextItemWidth(80);
    ImGui::DragFloat("step (deg)", &step, 0.5f, 0.5f, 90.0f, "%.1f");
    auto rotRow = [&](const char* label, const Eigen::Vector3d& axis) {
      ImGui::Text("%s", label);
      ImGui::SameLine();
      ImGui::PushID(label);
      if (ImGui::SmallButton("-")) s.applyWorldRotation(axis, -step);
      ImGui::SameLine();
      if (ImGui::SmallButton("+")) s.applyWorldRotation(axis, step);
      ImGui::PopID();
    };
    rotRow("Rot X", Eigen::Vector3d(1, 0, 0));
    rotRow("Rot Y", Eigen::Vector3d(0, 1, 0));
    rotRow("Rot Z", Eigen::Vector3d(0, 0, 1));
    if (ImGui::Button("Flip Up (180 X)", ImVec2(-FLT_MIN, 0)))
      s.applyWorldRotation(Eigen::Vector3d(1, 0, 0), 180.0);
    ImGui::EndDisabled();
  }

  ImGui::End();
}

void drawViewport(AppState& s) {
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::Begin("Viewport");
  ImGui::PopStyleVar();

  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const int w = (int)avail.x, h = (int)avail.y;
  if (w > 0 && h > 0) {
    s.fbo.resize(w, h);
    s.camera.setViewportSize(w, h);

    s.fbo.bind();
    glEnable(GL_DEPTH_TEST);
    glClearColor(s.bgColor.x(), s.bgColor.y(), s.bgColor.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const Eigen::Matrix4f view = s.camera.view();
    const Eigen::Matrix4f proj = s.camera.proj();
    s.meshRenderer.draw(view, proj, s.modelF(), s.partColor);

    // Datum planes (only meaningful once aligned): world XY/XZ/YZ through origin.
    if (s.hasResult && s.showDatums) {
      const float h = s.mesh.empty() ? 1.0f : (float)s.mesh.diagonal() * 0.5f;
      const Eigen::Vector3f O(0, 0, 0), X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
      s.planeRenderer.draw(view, proj, O, X, Y, h, Eigen::Vector4f(0.30f, 0.55f, 1.0f, 0.16f));  // Top
      s.planeRenderer.draw(view, proj, O, X, Z, h, Eigen::Vector4f(1.0f, 0.45f, 0.55f, 0.16f));  // Front
      s.planeRenderer.draw(view, proj, O, Y, Z, h, Eigen::Vector4f(0.45f, 0.9f, 0.55f, 0.16f));  // Right
    }

    // Picked-point markers (drawn on top, depth-test off).
    if (!s.pickedPoints.empty()) {
      const Eigen::Matrix4f model = s.modelF();
      const Eigen::Vector3f camR = view.row(0).head<3>();
      const Eigen::Vector3f camU = view.row(1).head<3>();
      const float msz = s.mesh.empty() ? 0.1f : (float)s.mesh.diagonal() * 0.012f;
      glDisable(GL_DEPTH_TEST);
      for (const auto& p : s.pickedPoints) {
        const Eigen::Vector4f w = model * Eigen::Vector4f((float)p.x(), (float)p.y(), (float)p.z(), 1);
        s.planeRenderer.draw(view, proj, w.head<3>(), camR, camU, msz,
                             Eigen::Vector4f(1.0f, 0.85f, 0.2f, 0.95f));
      }
      glEnable(GL_DEPTH_TEST);
    }

    if (s.showTriad) {
      float len = s.mesh.empty() ? 1.0f : (float)s.mesh.diagonal() * 0.55f;
      glDisable(GL_DEPTH_TEST);
      s.triad.draw(view, proj, len);
      glEnable(GL_DEPTH_TEST);
    }
    s.fbo.unbind();

    ImGui::Image((ImTextureID)(intptr_t)s.fbo.texture(), avail, ImVec2(0, 1), ImVec2(1, 0));
    const ImVec2 imgMin = ImGui::GetItemRectMin();

    // Datum gizmo (overlaid in screen space using the same camera matrices).
    bool gizmoUsing = false;
    if (s.showGizmo && (s.hasResult || s.userEdited)) {
      ImGuizmo::SetOrthographic(false);
      ImGuizmo::SetDrawlist();
      ImGuizmo::SetRect(imgMin.x, imgMin.y, avail.x, avail.y);
      Eigen::Matrix4f model = s.modelF();
      Eigen::Matrix4f v = view, p = proj;
      const ImGuizmo::OPERATION op =
          s.gizmoOp == 1 ? ImGuizmo::TRANSLATE : ImGuizmo::ROTATE;
      if (ImGuizmo::Manipulate(v.data(), p.data(), op, ImGuizmo::WORLD, model.data())) {
        s.setUserTransform(model.cast<double>());
      }
      gizmoUsing = ImGuizmo::IsUsing();
    }

    if (ImGui::IsItemHovered() && !gizmoUsing) {
      ImGuiIO& io = ImGui::GetIO();
      const bool picking = s.pickMode != AppState::PickMode::None;
      if (picking && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        // build a world-space ray from the cursor, then map into part space
        const float ndcx = (io.MousePos.x - imgMin.x) / avail.x * 2.0f - 1.0f;
        const float ndcy = 1.0f - (io.MousePos.y - imgMin.y) / avail.y * 2.0f;
        const Eigen::Matrix4f invVP = (proj * view).inverse();
        Eigen::Vector4f nh = invVP * Eigen::Vector4f(ndcx, ndcy, -1.0f, 1.0f);
        Eigen::Vector4f fh = invVP * Eigen::Vector4f(ndcx, ndcy, 1.0f, 1.0f);
        const Eigen::Vector3f nw = nh.head<3>() / nh.w();
        const Eigen::Vector3f fw = fh.head<3>() / fh.w();
        const Eigen::Vector3f dW = (fw - nw).normalized();
        const Eigen::Matrix4d invModel = s.currentTransform().inverse();
        const Eigen::Vector3d oP =
            (invModel * Eigen::Vector4d(nw.x(), nw.y(), nw.z(), 1.0)).head<3>();
        const Eigen::Vector3d dP = invModel.topLeftCorner<3, 3>() * dW.cast<double>();
        s.pickRay(oP, dP);
      } else if (!picking && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        s.camera.orbit(io.MouseDelta.x, io.MouseDelta.y);
      } else if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
                 ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
        s.camera.pan(io.MouseDelta.x, io.MouseDelta.y);
      }
      if (io.MouseWheel != 0.0f) s.camera.dolly(io.MouseWheel);
    }
    if (s.pickMode != AppState::PickMode::None && ImGui::IsKeyPressed(ImGuiKey_Escape))
      s.cancelPick();
  }
  ImGui::End();
}

}  // namespace ma::ui
