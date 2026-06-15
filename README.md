# MESH-Align

Cross-platform C++ desktop tool for the first, most tedious step of scan-to-CAD:
squaring a freshly imported scan to a clean coordinate frame. Drop in an
**STL/OBJ/PLY** and MESH-Align detects flat faces, bores, and symmetry, then
snaps the part to a clean **Top / Front / Right** datum frame — with manual datum
tools when it guesses wrong, and a deviation heatmap for checking a scan against
nominal.

Runs on **Windows, macOS, and Linux**. Tested with Revopoint Metro X Pro scans.

## Features

- **Import** STL / OBJ / PLY (drag-and-drop, file dialog, or CLI), with a mesh
  **validation** report (components, manifoldness, boundary edges, watertight).
- **Auto-orientation** — an analysis pipeline runs off the UI thread:
  normals → region-grow **plane extraction** → RANSAC **bore/cylinder detection**
  (with diameter + length) → reflective **symmetry** → **Top/Front/Right**
  assignment with a confidence score and the method used. Detected features are
  listed with a **classification** (Primary/Secondary/Tertiary · source, plus
  *bore-seat* flats), and the header shows top-datum confidence + the part's PCA
  elongation ratio. The part snaps to the datum frame with translucent datum planes.
- **Manual datum tools** — reassign any detected plane to Top/Front/Right; an
  inline **per-datum editor** (rotate about an axis + offset with live preview,
  **Apply** / **Reset**, per-datum flatness RMS); define a **plane from 3 picked
  points**; **click a hole** to fit a cylinder and align its axis; nudge per-axis;
  or drag a transform **gizmo**. *Revert to Auto* restores the automatic result.
- **Deviation heatmap** — best-fit **ICP** to a loaded reference mesh (or compare
  against a fitted datum plane), per-vertex **signed distance**, a **diverging
  blue→white→red** colormap with a green in-tolerance overlay, **auto/manual range**,
  a mm **scalar color bar**, and RMS + in-tolerance % readouts.
- **Export** the aligned mesh (STL/OBJ/PLY) and the 4×4 transform.

## Layout

```
core/    meshalign_core    geometry + data, NO GL / NO ImGui   (namespace ma)
render/  meshalign_render  OpenGL 3.3 renderer                 (namespace ma::gl)
app/     meshalign         Dear ImGui shell                    (namespace ma::ui)
tests/   headless core tests
docs/    design docs (PLAN / ARCHITECTURE / ALGORITHMS)
```

The geometry core links no GL/ImGui and is unit-tested headlessly. See
[AGENTS.md](AGENTS.md) for the architecture and contributor notes, and
[`docs/`](docs/) for the full design.

## Build

Requires CMake ≥ 3.20 and a C++17 compiler. Dependencies (Eigen, GLFW, Dear
ImGui + ImGuizmo, tinyobjloader, tinyply, nanoflann, portable-file-dialogs, and
glad off-macOS) are fetched automatically via CPM on first configure, so the
**first build needs network access**.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure      # headless tests
```

On Linux also install GL/X11 dev headers:
`sudo apt-get install libgl1-mesa-dev xorg-dev`.

### Options
- `-DMA_BUILD_APP=OFF` — build just the geometry core + tests (no GL).
- `-DMA_BUILD_TESTS=OFF` — skip tests.
- `-DMA_USE_SYSTEM_GLFW=ON` — link a system GLFW instead of fetching one.

## Run

```sh
./build/app/meshalign                                  # loads a built-in demo part
./build/app/meshalign part.stl                         # open a mesh on startup
./build/app/meshalign part.stl --analyze               # auto-orient on load
./build/app/meshalign scan.ply --ref nominal.ply --deviation   # heatmap on load
```

On **macOS** the build produces an app bundle (`build/app/meshalign.app`) with the
application icon — `open build/app/meshalign.app`, or run with CLI flags via
`./build/app/meshalign.app/Contents/MacOS/meshalign part.stl`. Windows/Linux build
a plain `meshalign(.exe)` with the icon embedded (taskbar) / installed (`.desktop`).

Sample meshes are generated into [`samples/`](samples/) for quick testing.

## Controls

- **Left-drag** orbit · **Middle/Right-drag** pan · **Wheel** zoom
- View presets (Top/Front/Right/Iso/Back/Left) in the right-hand *Datums* panel
- **Run Analysis** to auto-orient; reassign datums with the **T/F/R** buttons per
  detected plane
- **Plane from 3 Points** / **Pick Hole / Cylinder**: pick the target datum, then
  click in the viewport (**Esc** cancels)
- **Nudge Datum**: per-axis rotate, or enable the **Gizmo** to drag the frame
- **Deviation**: Open Reference → Run Deviation; drag the in-tol/warn sliders live
- **Ctrl/Cmd+Q** to quit

## Releases

Pushing a `v*` tag builds and tests on Linux x64, Windows x64, and macOS arm64;
if all succeed, the binaries are attached to the GitHub Release for that tag.

```sh
git tag v0.1.0 && git push origin v0.1.0
```

> Binaries are unsigned. On macOS, first launch may need
> `xattr -dr com.apple.quarantine ./meshalign` (or right-click → Open).

## Notes

- PLY: read (ASCII + binary) via tinyply 3.0; export is binary little-endian.
- ICP/deviation currently run synchronously on the click; large scans may pause
  briefly while computing.
