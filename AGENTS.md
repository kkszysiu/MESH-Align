# AGENTS.md

Guidance for AI agents (and humans) working in this repo.

## What this is

**MESH-Align** — a cross-platform C++ desktop app that imports a scan mesh
(STL/OBJ/PLY), auto-detects flat faces / bores / symmetry, and snaps the part to
a clean **Top/Front/Right** datum frame, with manual datum tools and a deviation
heatmap for checking a scan against nominal.

Design docs live in [`docs/`](docs/): `PLAN.md`, `ARCHITECTURE.md`, `ALGORITHMS.md`.
Read those before changing the analysis pipeline.

## Build / test / run

Requires CMake ≥ 3.20 and a C++17 compiler. **First configure needs network** —
dependencies are fetched via CPM (Eigen, GLFW, Dear ImGui+ImGuizmo,
tinyobjloader, tinyply, nanoflann, portable-file-dialogs; glad off-macOS).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure      # 5 headless suites
./build/app/meshalign                           # run (loads a demo part)
./build/app/meshalign samples/part_tilted.ply --analyze
./build/app/meshalign samples/part_scan.ply --ref samples/part.ply --deviation
```

CMake options: `-DMA_BUILD_APP=OFF` (core + tests only, no GL), `-DMA_BUILD_TESTS=OFF`,
`-DMA_USE_SYSTEM_GLFW=ON`. On Linux also install `libgl1-mesa-dev xorg-dev`.

Regenerate sample meshes: see `samples/` (procedurally produced; safe to delete).

## Architecture — the one rule that matters

Three CMake targets with a strict one-way dependency. **The geometry core must
not include or link any GL / GLFW / ImGui header.** This is enforced by a
headless test build and is the project's most important invariant.

```
core/    meshalign_core    geometry + data     namespace ma       (NO GL/ImGui)
render/  meshalign_render  OpenGL 3.3 renderer  namespace ma::gl   (GL only, no ImGui)
app/     meshalign         Dear ImGui shell     namespace ma::ui
```

Dependency direction: `app → render → core`. If you need geometry in the app,
put it in `core` and call it; never reach the other way.

- Data flows as value-typed DTOs in `core/include/ma/Types.h`.
- Analysis runs on a worker thread (`app/PipelineRunner.h`); **GL is only ever
  touched on the UI/main thread.** Results hand back via `poll()` each frame.
- The 3D viewport renders to an FBO texture shown as an `ImGui::Image`.

## Conventions

- Meshes use the libigl convention: `Eigen::MatrixXd V` (#V×3), `Eigen::MatrixXi F`
  (#F×3). Millimetres throughout.
- **double** for geometry/fitting math in `core`; **float** for GL buffers.
- New geometry algorithm → new `core/src/Foo.cpp` + `core/include/ma/Foo.h`, add
  to `core/CMakeLists.txt`, and add a headless test in `tests/` wired into
  `tests/CMakeLists.txt` + `ctest`.
- Match the surrounding style (2-space indent, `lowerCamelCase` functions,
  `PascalCase` types, anonymous namespaces for file-local helpers).

## Gotchas learned the hard way (don't re-discover these)

- **Eigen modules**: a `.a` compiles even if a templated symbol is undefined, then
  the final link fails. `Vector3d::cross` needs `#include <Eigen/Geometry>`;
  `Matrix::inverse()` needs `#include <Eigen/LU>`. Include them in the `.cpp` that
  uses them.
- **OpenGL loading**: macOS links the system GL framework (`<OpenGL/gl3.h>`, no
  loader); other platforms use glad (`<glad/gl.h>`). Always include via
  `ma_gl/GL.h`, never a GL header directly.
- **tinyply 2.3.4's ASCII reader is broken.** We write **binary** PLY and have a
  hand-rolled ASCII PLY reader in `core/src/io/LoadPLY.cpp`. Don't route ASCII PLY
  through tinyply.
- **portable-file-dialogs**: the default-path arg is a **folder**. Passing a
  filename or `"."` makes the backend error and show nothing — pass an empty
  string (platform default) or a real directory. Dialogs are **non-blocking**
  (polled in `AppState::pollDialogs`) so they don't open behind a frozen window.
  pfd is cross-platform (osascript on macOS, Win32 on Windows, zenity/kdialog on
  Linux — Linux needs one installed).
- **A flat quad face is 2 triangles**, not 3+ — plane/region thresholds must allow
  small regions. Region-grow needs **shared vertices**; procedural primitives that
  split vertices per face won't be adjacency-connected.
- **Pinned deps**: ImGui (docking) and ImGuizmo are pinned to exact SHAs in
  `cmake/dependencies.cmake`; ImGuizmo sources live under `src/`. ImGui renamed
  `ImGuiChildFlags_Border` → `_Borders`.

## Status

M0–M4 complete: import → validate → auto-orient → manual datums → deviation
heatmap. Open follow-ups: move ICP/deviation to the worker thread (currently
synchronous), wire the planned "Apply Adjustment", real CI run, and packaging
(M5). CI (`.github/workflows/build.yml`) is written but unverified across the 3
OSes; verification so far is macOS-only.

## When you change things

- Run `ctest` and keep all suites green.
- If you touch the analysis pipeline, update `docs/ALGORITHMS.md`.
- Commit messages: see git history for style. End commit messages with the
  `Co-Authored-By` trailer if the change was AI-assisted. Only commit/push when
  asked.
