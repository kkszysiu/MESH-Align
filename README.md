# MESH-Align

Auto-alignment for scan meshes. Drop in an STL/OBJ/PLY, and MESH-Align detects
flat faces, bores, and symmetry, then snaps the part to a clean **Top / Front /
Right** datum frame — with manual datum tools and a deviation heatmap for checking
a scan against nominal.

See [`docs/PLAN.md`](docs/PLAN.md), [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md),
and [`docs/ALGORITHMS.md`](docs/ALGORITHMS.md) for the full design.

> Status: **M3 — manual datum tools.** Everything from M2 plus full manual
> override: reassign any detected plane to Top/Front/Right, **3-point plane**
> picking, **click-a-hole cylinder** fit (ray-cast → smooth-region grow →
> local RANSAC), nudge (per-axis rotate / flip), and an **ImGuizmo** rotate/
> move gizmo on the datum frame. "Revert to Auto" restores the auto result.
> The deviation heatmap (M4) follows.
>
> Run analysis on startup with the `--analyze` flag.
>
> Note: PLY export is binary little-endian; ASCII PLY input is handled by a
> small built-in reader (tinyply 2.3.4's ASCII path is unreliable).

## Layout

```
core/    meshalign_core    geometry + data, NO GL / NO ImGui   (namespace ma)
render/  meshalign_render   OpenGL 3.3 renderer                 (namespace ma::gl)
app/     meshalign          Dear ImGui shell                    (namespace ma::ui)
tests/   headless core tests
docs/    design docs
```

## Build

Requires CMake ≥ 3.20 and a C++17 compiler. Dependencies (Eigen, GLFW, Dear ImGui
+ ImGuizmo, and glad off-macOS) are fetched automatically via CPM on first
configure, so the first build needs network access.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/app/meshalign                 # run the app (demo part)
./build/app/meshalign part.stl        # ...or open a mesh on startup
ctest --test-dir build                # run headless core tests
```

On Linux you also need GL/X11 dev headers: `sudo apt-get install libgl1-mesa-dev xorg-dev`.

### Useful options
- `-DMA_BUILD_APP=OFF` — build just the geometry core + tests (no GL).
- `-DMA_USE_SYSTEM_GLFW=ON` — link a system GLFW instead of fetching.

## Controls
- **Left-drag** orbit · **Middle/Right-drag** pan · **Wheel** zoom
- View presets (Top/Front/Right/Iso/Back/Left) in the right-hand *Datums* panel
- **Run Analysis** to auto-orient; reassign datums with the **T/F/R** buttons
  per detected plane
- **Plane from 3 Points** / **Pick Hole / Cylinder**: choose the target datum,
  click in the viewport (**Esc** cancels)
- **Nudge Datum**: per-axis rotate, or enable the **Gizmo** to drag the frame
