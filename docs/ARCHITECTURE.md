# MESH-Align — Architecture

## Layering

Three hard layers with a one-directional dependency. **The geometry core links no
ImGui, GLFW, or OpenGL header.** This is the single most important invariant: it
gives headless unit tests on real scan files in CI and decouples algorithm work
from UI churn. Enforced via separate CMake targets + a CI build of
`meshalign_core` with zero GL/ImGui linkage.

```
meshalign/
  core/        static lib  meshalign_core    — NO UI, NO GL.   namespace ma
  render/      static lib  meshalign_render   — GL only, no ImGui. namespace ma::gl
  app/         executable  meshalign          — ImGui glue.     namespace ma::ui
  third_party/ CPM-fetched, pinned SHAs
  tests/       headless core unit tests + a test-mesh corpus
  assets/      shaders, matcap PNG, fonts
  docs/        PLAN / ARCHITECTURE / ALGORITHMS
```

Dependency direction: `app → render → core`. `core` depends only on Eigen,
nanoflann, and the header-only IO loaders (tinyobjloader / tinyply). (The original
plan considered libigl; we implemented the few pieces we needed in-house instead.)

---

## `core` (namespace `ma`)

Pure data + algorithms. Everything operates on Eigen matrices (`Eigen::MatrixXd V`
N×3, `Eigen::MatrixXi F` M×3) — the standard libigl-style V/F layout. Double
precision for fitting math.

| Type | Responsibility |
|---|---|
| `ma::Mesh` | owns `V`, `F`, vertex/face normals, bbox, units |
| `ma::io` | `loadMesh(path)→Mesh`, `saveMesh(path,mesh,fmt)`, `saveTransform(path,Mat4)`; dispatch by extension |
| `ma::MeshValidation` | component count, non-manifold edges, degenerate/duplicate verts → `ValidationReport` (flag only, never mutate the user mesh) |
| `ma::Normals` | `estimateFaceNormals` (primary for RANSAC), `estimateVertexNormals(k)` |
| `ma::PlaneExtractor` | RANSAC + region-grow → `std::vector<DetectedPlane>` |
| `ma::CylinderExtractor` | bore/hole detection → `std::vector<DetectedCylinder>` |
| `ma::SymmetryDetector` | reflective symmetry plane(s) |
| `ma::Orienter` | assign Top/Front/Right, origin, confidence, method, PCA ratio → `OrientationResult`; `orientFromAxes()` rebuilds a frame from explicit Z/X/origin (manual overrides) |
| `ma::Geometry` | `computeFaceData` (normals/areas/centroids), `triangleAdjacency` (own edge map), `weightedPCA` |
| `ma::Picking` | `raycast` (Möller–Trumbore) + `growSmoothRegion` (bore-wall flood) |
| `ma::Bvh` | AABB tree over triangles → `closest(point)` for deviation/ICP |
| `ma::Deviation` | `icpToReference` (point-to-plane), `deviationToReference`/`deviationToPlane` (signed distance), `planeResidual` (per-datum flatness) |
| `ma::AnalysisPipeline` | staged orchestration; progress callback; single entry the app calls on a worker; also tags bore-seat candidates |

### DTOs (UI-agnostic, copyable, value-typed)

```cpp
struct DetectedPlane    { Eigen::Vector3d point, normal; double confidence, area;
                          std::vector<int> inlierFaces; DatumRole role;
                          std::string label, tier, source; };   // tier/source = classification
struct DetectedCylinder { Eigen::Vector3d axisPoint, axisDir; double radius, length, confidence;
                          std::vector<int> inlierFaces; std::string label; };
struct SymmetryPlane    { Eigen::Vector3d point, normal; double score; };
struct OrientationResult{ Eigen::Matrix4d transform;        // part -> world (datum frame)
                          Eigen::Vector3d eulerXYZ_deg;     // display only
                          Eigen::Vector3d pcaRatio;         // shape elongation largest:mid:1
                          double confidence; std::string method;
                          int topPlaneId, frontPlaneId, rightPlaneId; };
struct ToleranceBands   { double nominal = 0, inTolMm, warnMm; };
struct DeviationField   { Eigen::VectorXd signedDistMm; double minMm, maxMm;
                          ToleranceBands bands; };
struct AnalysisResult   { ValidationReport validation;
                          std::vector<DetectedPlane> planes;
                          std::vector<DetectedCylinder> cylinders;
                          std::vector<SymmetryPlane> symmetry;
                          OrientationResult orientation; };
```

Note: we ended up **not** taking on libigl — adjacency (`triangleAdjacency`), the
closest-point BVH (`ma::Bvh`), and ray casting (`ma::Picking`) are all small
in-house implementations, keeping the dependency set lean (Eigen + nanoflann for
the core). Revisit libigl only if a heavier mesh operation is needed.

---

## `render` (namespace `ma::gl`)

Pure GL, fed by core DTOs + a transform. No knowledge of ImGui.

| Type | Responsibility |
|---|---|
| `MeshRenderer` | VAO/VBO upload; matcap shader; `drawHeatmap` per-vertex scalar path sampling a 1D LUT |
| `Heatmap` | builds the band-LUT texture + `bandColor` (diverging blue→white→red, green in-tol); shared by shader and scalar bar |
| `PlaneRenderer` | translucent depth-sorted quads for datum planes + picked-point markers |
| `TriadRenderer` | origin axis triad |
| `Camera` | arcball orbit/pan/zoom; view presets Top/Front/Right/Iso/Back/Left → view matrices |
| `Framebuffer` | render the 3D scene to a texture so the viewport is an `ImGui::Image` inside a dock panel |

Rendering to an FBO texture (rather than the main framebuffer) avoids scissor-rect
reasoning and makes the viewport a normal dock panel.

---

## `app` (namespace `ma::ui`)

| Type | Responsibility |
|---|---|
| `AppState` | **single source of truth**: `Mesh`, `AnalysisResult`, reference mesh, selection state, view options, worker handle, progress snapshot |
| panel functions | free functions taking `AppState&`: `drawMenuBar/drawLeftPanel/drawViewport/drawRightPanel/drawLog` (immediate-mode = no widget objects) |
| `GizmoController` | wraps ImGuizmo; reads/writes the active datum transform in `AppState` |
| `PickController` | turns mouse rays into core picking calls |
| `PipelineRunner` | owns the worker thread, the progress channel, and result hand-off |

Target panel layout: top menu bar (File/Edit/Analyze/View/Settings/
Help); left panel (file/stats/analysis/pipeline/orientation result/log); center
viewport (datum planes + origin triad + transform gizmo); right panel (view
presets, detected-planes list, actions: Export aligned mesh / Export transform /
Revert to Auto / Plane from 3 points / Pick hole-cylinder).

---

## Data flow

```
File drop ─▶ ma::io::loadMesh ─▶ AppState.mesh ─▶ MeshRenderer upload
   │
   └─(Run Analysis)─▶ PipelineRunner spawns worker
            worker: AnalysisPipeline.run(mesh, progressCb) ─▶ AnalysisResult
            progressCb ─▶ {stage, pct, msg} into a guarded snapshot
   UI thread/frame: poll snapshot ─▶ pipeline panel + log
   on completion: future ready ─▶ AppState.result ─▶ renderers refreshed
   Manual override (gizmo / pick) ─▶ mutates OrientationResult.transform / planes
            ─▶ recompute dependent display only (no full re-run)
   Export ─▶ ma::io::saveMesh(applyTransform(mesh, transform))
```

**Golden rule:** UI never blocks on core; core never calls UI. Communication is
(a) a value-typed `AnalysisResult` returned via `std::future`, and (b) a progress
snapshot polled each frame.

---

## Threading

- One **worker thread** for the analysis pipeline; UI stays at 60 fps.
- Progress: pipeline takes `std::function<void(Stage, float pct, std::string msg)>`
  writing a mutex/atomic-guarded snapshot the UI polls per frame. **Never call
  ImGui from the worker.**
- Result hand-off: worker produces a fully-formed `AnalysisResult` value; UI checks
  `future.wait_for(0)` each frame, then moves it into `AppState` and triggers GL
  uploads (**GL touched only on the UI thread**).
- Cancellation: `std::atomic<bool> cancel` checked between/within stages, wired to a
  Cancel button.
- Stage granularity for the panel: Validation → Normals → Planes → Cylinders →
  Symmetry → Orientation (→ Deviation if a reference is loaded).
- Build heavy structures (face data, adjacency, `ma::Bvh`) on the worker; picking on
  the UI thread reuses them read-only.
- (Future) OpenMP for inner loops (normals, signed distance, RANSAC scoring) once
  meshes are large enough to warrant it.

---

## Build & dependency strategy

**CPM.cmake (FetchContent wrapper) for everything; skip vcpkg for this dep set.**
Only GLFW is a "real" compiled library; the rest are header-only or single-TU, so
source-based fetching with pinned SHAs wins (reproducible, no external tooling,
identical across OSes).

- `CPM.cmake` for all third-party, each pinned to a SHA/tag.
- **GLFW:** fetch + build static, with a `MA_USE_SYSTEM_GLFW` option (Linux distro
  package fallback).
- **Header-only deps:** Eigen, nanoflann, tinyobjloader, tinyply, Dear ImGui +
  ImGuizmo (built into one static lib), portable-file-dialogs.
- **glad:** GL 3.3 core loader, fetched off-macOS only (macOS uses the system GL
  framework).
- GL context: request 3.3 core forward-compatible (works on Win/Linux and macOS,
  which caps at 4.1).

### Packaging (CPack per platform)
- Windows → NSIS installer or zipped portable folder.
- macOS → `.app` bundle (assets in `Resources/`, rpath set), optionally `.dmg`;
  budget time for code-signing/notarization.
- Linux → AppImage ("drop a binary") + `.tar.gz`.
- Bundle assets (shaders, matcap PNG, fonts) and resolve paths relative to the
  executable, not CWD.

---

## Critical files to create (dependency order)

- `core/include/ma/AnalysisPipeline.h` — orchestration entry + DTO contract between core and app.
- `core/src/Orienter.cpp` — Top/Front/Right assignment + origin + confidence (highest-risk code).
- `core/src/PlaneExtractor.cpp`, `core/src/CylinderExtractor.cpp` — RANSAC + region-grow fitting.
- `core/src/Deviation.cpp` — ICP + signed distance + banding.
- `app/AppState.h` — single source of truth across panels/gizmo/picking/runner.
- `CMakeLists.txt` (+ `cmake/CPM.cmake`) — the cross-platform build/dep graph.
