#include "ma_gl/GL.h"   // GL headers first

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <ImGuizmo.h>

#include "AppState.h"
#include "Panels.h"
#include "ma/PrimitiveMeshes.h"

#include <cstdio>

using ma::ui::AppState;

namespace {

void glfwErrorCallback(int error, const char* desc) {
  std::fprintf(stderr, "[glfw] error %d: %s\n", error, desc);
}

// Load the last dropped file (GL context is current on the main thread here).
void dropCallback(GLFWwindow* window, int count, const char** paths) {
  auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
  if (app && count > 0) app->loadMeshFile(paths[count - 1]);
}

// One-time default dock layout matching the mockup: thin left + right panels,
// big viewport in the middle.
void buildDefaultLayout(ImGuiID dockspaceId) {
  ImGui::DockBuilderRemoveNode(dockspaceId);
  ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockspaceId, ImGui::GetMainViewport()->WorkSize);

  ImGuiID center = dockspaceId;
  ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.22f, nullptr, &center);
  ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.28f, nullptr, &center);

  ImGui::DockBuilderDockWindow("Inspector", left);
  ImGui::DockBuilderDockWindow("Datums", right);
  ImGui::DockBuilderDockWindow("Viewport", center);
  ImGui::DockBuilderFinish(dockspaceId);
}

}  // namespace

int main(int argc, char** argv) {
  const char* startupMesh = nullptr;
  const char* refMesh = nullptr;
  bool autoAnalyze = false;
  bool autoDeviation = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--analyze") autoAnalyze = true;
    else if (a == "--deviation") autoDeviation = true;
    else if (a == "--ref" && i + 1 < argc) refMesh = argv[++i];
    else if (a.rfind("--", 0) != 0 && !startupMesh) startupMesh = argv[i];
  }
  glfwSetErrorCallback(glfwErrorCallback);
  if (!glfwInit()) {
    std::fprintf(stderr, "Failed to init GLFW\n");
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

  GLFWwindow* window = glfwCreateWindow(1440, 810, "MESH-Align", nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "Failed to create window\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

#ifdef MA_USE_GLAD
  if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
    std::fprintf(stderr, "Failed to load GL via glad\n");
    return 1;
  }
#endif
  std::fprintf(stderr, "GL: %s\n", glGetString(GL_VERSION));

  // ---- ImGui ----
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.IniFilename = nullptr;  // deterministic layout for M0
  ImGui::StyleColorsDark();
  ImGui::GetStyle().FrameRounding = 3.0f;
  ImGui::GetStyle().WindowRounding = 4.0f;

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  // ---- App ----
  AppState app;
  if (!app.meshRenderer.init() || !app.triad.init() || !app.planeRenderer.init()) {
    std::fprintf(stderr, "Renderer init failed\n");
    return 1;
  }
  app.lutTex = ma::gl::makeBandLutTexture();
  app.refreshLut();
  glfwSetWindowUserPointer(window, &app);
  glfwSetDropCallback(window, dropCallback);

  app.camera.setPreset(ma::gl::Camera::Preset::Iso);
  if (startupMesh && app.loadMeshFile(startupMesh)) {
    // loaded from CLI
  } else {
    app.mesh = ma::makeDemoPart();
    app.meshRenderer.upload(app.mesh);
    app.validation = ma::validate(app.mesh);
    app.hasValidation = true;
    app.frameMesh();
    app.logLine("MESH-Align M2 - drop a mesh, then Run Analysis");
  }
  if (refMesh) app.loadReference(refMesh);
  if (autoAnalyze) app.runAnalysis();
  if (autoDeviation) app.runDeviation();

  bool layoutBuilt = false;
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Hand a finished analysis result back to the UI thread.
    ma::AnalysisResult pending;
    if (app.pipeline.poll(pending)) app.applyResult(std::move(pending));

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGuizmo::BeginFrame();

    ImGuiID dockId = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
    if (!layoutBuilt) {
      buildDefaultLayout(dockId);
      layoutBuilt = true;
    }

    ma::ui::drawMenuBar(app);
    ma::ui::drawLeftPanel(app);
    ma::ui::drawRightPanel(app);
    ma::ui::drawViewport(app);

    ImGui::Render();
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glViewport(0, 0, fbw, fbh);
    glClearColor(0.10f, 0.10f, 0.11f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
