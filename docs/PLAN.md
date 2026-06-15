# MESH-Align — Implementation Plan

## Context

Scan-to-CAD work always starts with the same chore: a fresh 3D scan (e.g. from a
**Revopoint Metro X Pro**) imports at an arbitrary angle, and before you can
measure or model you have to square it to a clean coordinate frame by eye — slow
and fiddly. **MESH-Align** automates that first step. You drop in an STL/OBJ/PLY,
it detects flat faces, bores, and symmetry, and snaps the part to a clean
**Top/Front/Right** datum frame. When the auto-guess is wrong, you reassign or
nudge any datum by hand (3-point planes, click-a-hole cylinder-axis alignment). A
**deviation heatmap** with tolerance bands checks a scan against nominal.

The intended UI and flows: drop → **Run Analysis** → pipeline (validation →
normal estimation → auto-orientation → plane assignment) → snap to datums with an
**Orientation Result** (rotation X/Y/Z, confidence, method) and a **Detected
Planes** list; plus a deviation heatmap mode with a mm color scale, tolerance
bands, and **Apply Adjustment**.

**This is a greenfield project** — everything below is to be created.

### Decisions locked with the user
- **Stack:** C++ with Dear ImGui (immediate-mode GUI).
- **Nominal source for heatmap:** BOTH — best-fit (ICP) vs a loaded reference
  CAD/mesh, AND deviation vs the part's own fitted symmetry/datums.
- **MVP scope:** auto-align AND deviation heatmap together (one milestone arc).
- **Distribution:** desktop only — Windows / macOS / Linux.

See [`ARCHITECTURE.md`](./ARCHITECTURE.md) for module/layer decomposition and data
flow, and [`ALGORITHMS.md`](./ALGORITHMS.md) for the orientation + deviation math.

---

## Tech stack

| Concern | Choice | Notes |
|---|---|---|
| Window / GL context | **GLFW** | only "real" compiled dep; request GL 3.3 core, forward-compat (macOS-safe) |
| GL loader | **glad** (GL 3.3 core) | ImGui GL3 backend needs a loader |
| GUI | **Dear ImGui** (docking branch) | docked panels, viewport rendered to FBO shown via `ImGui::Image` |
| Gizmo | **ImGuizmo** | isolated behind `GizmoController` (replaceable) |
| Linear algebra | **Eigen** | PCA via `SelfAdjointEigenSolver`, SVD, transforms (double in core) |
| Mesh ops / BVH | _planned libigl; **implemented in-house**_ | adjacency, AABB-tree closest-point (`ma::Bvh`), ray cast, signed distance — small own code, no libigl dependency |
| kd-tree | **nanoflann** | kNN normals, RANSAC neighbor queries |
| Mesh IO | **tinyobjloader** (OBJ), **tinyply** (PLY), small custom STL reader | avoid assimp (heavy, lossy on raw scans); custom writers for export |
| Build / deps | **CMake + CPM.cmake** | pinned SHAs, no vcpkg (deps are header-only except GLFW) |
| Packaging | **CPack** | Win: NSIS/zip; macOS: `.app`/`.dmg`; Linux: AppImage/tar.gz |
| Threading | `std::thread` + `std::future` + atomic progress snapshot | analysis off the UI thread |

**Hard rule:** the geometry core links **no** ImGui/GLFW/GL. Enforced by separate
CMake targets + a headless CI build of `meshalign_core` with unit tests.

---

## Milestones (MVP includes BOTH auto-align and heatmap)

- **M0 — Skeleton (1–1.5 wk):** CMake+CPM, all deps building on 3 OSes. ImGui
  docking shell with the target panel layout; viewport-as-FBO; matcap render of a
  hardcoded mesh; camera + view presets; CI builds core headless + app.
- **M1 — IO & viewing (1 wk):** STL/OBJ/PLY loaders, drag-drop, stats panel,
  validation report, render real scans, save mesh + transform. Demoable.
- **M2 — Orientation core (2–3 wk, riskiest):** normals → planes (RANSAC+grow) →
  detected-planes list + translucent quads; cylinders; symmetry; Orienter
  (transform/Euler/confidence/method); PipelineRunner threading + progress panel.
- **M3 — Manual override + picking (1–1.5 wk):** datum gizmo + nudge, reassign
  Top/Front/Right, 3-point plane, click-hole cylinder fit (ray-cast via AABB).
- **M4 — Deviation/heatmap (2 wk):** reference load, robust point-to-plane ICP,
  BVH closest-point signed distance, tolerance bands, shader-LUT vertex coloring,
  scalar bar, mode-b vs own datums. Reuses M2 solvers + the BVH.
- **M5 — Export, packaging, polish (1–1.5 wk):** aligned-mesh + 4x4 transform
  export, CPack/AppImage/.app/.dmg, asset bundling, test-corpus regression,
  confidence/UX tuning.

Rough order: **~9–12 weeks** to a solid MVP.

---

## Top risks
1. **Orientation heuristics are the real hard problem** — "which plane is Top" is
   judgment. Mitigate: everything user-overridable from day one; honest
   confidence; a regression corpus of real triple-clamp scans + synthetic
   ground-truth parts.
2. **Scan messiness** (holes/noise/non-manifold/islands, multi-million-triangle
   meshes) — robust ICP rejection, area-weighted fits, hole-tolerant signed
   distance; validate on real Revopoint output early.
3. **Big-mesh performance** — ICP/deviation are O(verts) per iteration via the BVH;
   move them to the worker thread and add OpenMP when meshes get large.
4. **ImGuizmo quirks** — isolated behind `GizmoController`.
5. **Units/precision** — mm everywhere; double in core math, float in GL buffers;
   centralize at IO.
6. **macOS GL deprecation** — keep renderer thin behind `ma::gl` for a future
   backend swap.

---

## Verification
- **Headless core tests** (`tests/`, run in CI, no GL): load synthetic meshes with
  known ground-truth datums (a box, a box+bore, a symmetric bracket); assert
  detected plane normals, cylinder axis/radius, symmetry plane, and final
  Top/Front/Right transform within tolerance; assert PCA fallback path.
- **Deviation tests:** known-offset reference vs scan → assert signed distances and
  band classification match expected.
- **Manual E2E:** build the app, drop the reference triple-clamp scan, Run
  Analysis, confirm it snaps to Top/Front/Right with a confidence/method readout;
  verify manual reassign, 3-point plane, click-hole cylinder, and the gizmo nudge;
  load a nominal mesh, confirm ICP + heatmap + tolerance bands + scalar bar; export
  aligned mesh + transform and re-import to confirm orientation persisted.
- **Cross-platform smoke:** package and launch on Win/macOS/Linux.

---

## First steps on approval
0. Write `docs/PLAN.md`, `docs/ARCHITECTURE.md`, `docs/ALGORITHMS.md` (this step).
1. Scaffold the CMake/CPM project + target separation + CI (M0).
2. Stand up the ImGui docking shell and FBO viewport.
