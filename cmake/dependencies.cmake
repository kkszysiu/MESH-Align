# Third-party dependencies, fetched & pinned via CPM.
# Geometry core needs only Eigen. The GUI deps are added only when MA_BUILD_APP.

# ---- Eigen (header-only; download only, no Eigen CMake/tests) ----------------
CPMAddPackage(
  NAME eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
  DOWNLOAD_ONLY YES
)
add_library(eigen INTERFACE)
target_include_directories(eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")

# ---- Mesh IO loaders (header-based; usable by the core, no GL) ---------------
CPMAddPackage(
  NAME tinyobjloader
  GITHUB_REPOSITORY tinyobjloader/tinyobjloader
  GIT_TAG v2.0.0rc13
  DOWNLOAD_ONLY YES
)
add_library(tinyobjloader INTERFACE)
target_include_directories(tinyobjloader SYSTEM INTERFACE "${tinyobjloader_SOURCE_DIR}")

CPMAddPackage(
  NAME tinyply
  GITHUB_REPOSITORY ddiakopoulos/tinyply
  GIT_TAG 2.3.4
  DOWNLOAD_ONLY YES
)
add_library(tinyply INTERFACE)
target_include_directories(tinyply SYSTEM INTERFACE "${tinyply_SOURCE_DIR}/source")

# ---- nanoflann (kd-tree, header-only) ---------------------------------------
CPMAddPackage(
  NAME nanoflann
  GITHUB_REPOSITORY jlblancoc/nanoflann
  GIT_TAG v1.5.5
  DOWNLOAD_ONLY YES
)
add_library(nanoflann INTERFACE)
target_include_directories(nanoflann SYSTEM INTERFACE "${nanoflann_SOURCE_DIR}/include")

if(NOT MA_BUILD_APP)
  return()
endif()

# ---- OpenGL -----------------------------------------------------------------
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# ---- Native file dialogs (header-only, no deps) -----------------------------
CPMAddPackage(
  NAME portable_file_dialogs
  GITHUB_REPOSITORY samhocevar/portable-file-dialogs
  GIT_TAG 0.1.0
  DOWNLOAD_ONLY YES
)
add_library(portable_file_dialogs INTERFACE)
target_include_directories(portable_file_dialogs SYSTEM INTERFACE "${portable_file_dialogs_SOURCE_DIR}")

# ---- GLFW -------------------------------------------------------------------
if(MA_USE_SYSTEM_GLFW)
  find_package(glfw3 REQUIRED)
else()
  CPMAddPackage(
    NAME glfw
    GITHUB_REPOSITORY glfw/glfw
    GIT_TAG 3.4
    OPTIONS
      "GLFW_BUILD_DOCS OFF"
      "GLFW_BUILD_TESTS OFF"
      "GLFW_BUILD_EXAMPLES OFF"
      "GLFW_INSTALL OFF"
  )
endif()

# ---- glad (only needed off-Apple; macOS uses the system GL framework) --------
if(NOT APPLE)
  CPMAddPackage(
    NAME glad
    GITHUB_REPOSITORY Dav1dde/glad
    GIT_TAG v2.0.6
    DOWNLOAD_ONLY YES
  )
  add_subdirectory("${glad_SOURCE_DIR}/cmake" glad_cmake)
  glad_add_library(glad_gl STATIC API gl:core=3.3)
endif()

# ---- Dear ImGui (docking branch) + ImGuizmo, built into one static lib -------
# Pinned to the exact commits verified to build (docking / master tips).
CPMAddPackage(
  NAME imgui
  GITHUB_REPOSITORY ocornut/imgui
  GIT_TAG 2af6dd9694288e6befe1edb7ce25510911693c22  # docking, 2026-xx
  DOWNLOAD_ONLY YES
)
CPMAddPackage(
  NAME imguizmo
  GITHUB_REPOSITORY CedricGuillemet/ImGuizmo
  GIT_TAG be8aa4aeab86b402701c8c1df011bd8cd776760b  # master
  DOWNLOAD_ONLY YES
)

add_library(imgui STATIC
  "${imgui_SOURCE_DIR}/imgui.cpp"
  "${imgui_SOURCE_DIR}/imgui_draw.cpp"
  "${imgui_SOURCE_DIR}/imgui_tables.cpp"
  "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
  "${imgui_SOURCE_DIR}/imgui_demo.cpp"
  "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
  "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
  "${imguizmo_SOURCE_DIR}/src/ImGuizmo.cpp"
)
target_include_directories(imgui SYSTEM PUBLIC
  "${imgui_SOURCE_DIR}"
  "${imgui_SOURCE_DIR}/backends"
  "${imguizmo_SOURCE_DIR}/src"
)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
target_compile_definitions(imgui PUBLIC IMGUI_DEFINE_MATH_OPERATORS)
if(APPLE)
  target_compile_definitions(imgui PUBLIC GL_SILENCE_DEPRECATION)
endif()
