cmake_minimum_required(VERSION 3.20)
project(MeshAlign VERSION 0.1.0 LANGUAGES C CXX)

# ---- standards / defaults ---------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# On Windows, <windows.h> (pulled in by portable-file-dialogs / glad) defines
# min/max macros that collide with std::min/std::max and corrupt the parse.
# Suppress them everywhere (applies to all targets added below, incl. deps).
# (Only NOMINMAX — WIN32_LEAN_AND_MEAN would strip headers pfd needs.)
if(WIN32)
  add_compile_definitions(NOMINMAX)
endif()

# ---- options ----------------------------------------------------------------
option(MA_BUILD_APP   "Build the MESH-Align GUI app" ON)
option(MA_BUILD_TESTS "Build headless core tests"    ON)
option(MA_USE_SYSTEM_GLFW "Use a system-installed GLFW instead of fetching" OFF)

# ---- dependencies (CPM) -----------------------------------------------------
include(get_cpm)
include(dependencies)

# ---- targets ----------------------------------------------------------------
add_subdirectory(core)            # meshalign_core  — NO GL, NO ImGui

if(MA_BUILD_APP)
  add_subdirectory(render)        # meshalign_render — GL only
  add_subdirectory(app)           # meshalign        — ImGui shell
endif()

if(MA_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ---- summary ----------------------------------------------------------------
message(STATUS "MESH-Align ${PROJECT_VERSION}")
message(STATUS "  build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  app        : ${MA_BUILD_APP}")
message(STATUS "  tests      : ${MA_BUILD_TESTS}")
