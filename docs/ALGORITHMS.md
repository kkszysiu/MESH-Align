# MESH-Align — Algorithms

All math runs in `meshalign_core` on Eigen matrices (`V` N×3, `F` M×3), double
precision. Distances are in millimetres. Every stage must tolerate non-watertight,
noisy, multi-component scan input.

---

## Auto-orientation pipeline

### 1. Mesh validation
- Component count via a union-find over edge-connected faces.
- Boundary / non-manifold edges via edge-incidence counts (count==1 / count>2).
- Degenerate faces (zero area), duplicate verts (quantized-coordinate hash).
- **Flag only — never auto-modify the user's mesh.** Downstream stages are designed
  hole-tolerant; watertightness is not assumed.

### 2. Normal estimation
- **Face normals** are the primitive for plane RANSAC (exact from triangle
  geometry), area-weighted.
- Area-weighted vertex normals (computed from faces).
- For point-based ops: nanoflann kd-tree on `V`, gather k≈18–30 neighbors, build the
  3×3 covariance, `Eigen::SelfAdjointEigenSolver`, smallest eigenvector = normal.
  Orient consistently away from the centroid (MST propagation is the rigorous fix;
  centroid heuristic suffices for MVP).

### 3. Plane extraction — hybrid RANSAC + region growing
Pure RANSAC over millions of triangles is slow and merges coplanar-but-disjoint
faces. Use the Schnabel "Efficient RANSAC" idea adapted to a mesh:

1. **RANSAC seed** on face centroids+normals: sample 3 faces, accept a candidate
   only if their normals agree (`dot > cos(θ)`, θ≈10°). Score inliers by point-plane
   distance < ε (ε≈0.3–0.5 mm, scaled to bbox) **and** normal agreement.
2. **Region grow** from the best candidate: flood across
   `triangle_triangle_adjacency`, adding neighbor faces whose normal stays within
   the angle band and whose centroid stays within the distance band. Recovers full
   connected flat faces; won't bleed across a fillet.
3. **PCA refit** on the grown inlier vertices (area-weighted covariance):
   `SelfAdjointEigenSolver<Matrix3d>`; eigenvalues ascending → `eigenvectors().col(0)`
   is the refined normal, the mean is the plane point.
4. Remove inliers, repeat until remaining flat area < threshold or N planes found.
5. **Per-plane confidence** = normalized weighted blend of: inlier area / candidate
   area; RMS point-plane residual (lower better); absolute area (bigger = more
   trustworthy datum); planarity ratio λ_min/λ_mid (smaller = genuinely planar).

Eigen sketch:
```cpp
Eigen::Matrix3d M = (P.rowwise() - mean).transpose() * W.asDiagonal() * (P.rowwise() - mean);
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(M);
Eigen::Vector3d normal = es.eigenvectors().col(0);   // smallest eigenvalue
```

### 4. Cylinder / bore detection
Bores are the strongest datum for a triple clamp (steerer-tube + fork bores).

- **Candidate generation** among faces not assigned to planes. Cylinder normals all
  lie in a plane through the origin in normal-space, so PCA of the region's face
  normals yields a near-zero eigenvalue whose eigenvector ≈ the cylinder axis.
- **RANSAC fit:** pick 2 points with normals; `axis = (n1 × n2).normalized()`;
  project points onto the plane ⟂ axis; fit a 2D circle by algebraic (Kåsa) least
  squares — a linear system in Eigen — for center + radius. Score inliers by radial
  distance to the surface and radial-vs-normal agreement. Keep the best; optional
  Gauss-Newton refine of (axisPoint, axisDir, radius); skip refine for MVP.
- nanoflann localizes RANSAC sampling (cylinders are local features).
- **Axial length** = span of the inlier *vertices* projected onto the axis (using
  centroids underestimates, since they cluster mid-height). Reported with the
  radius as diameter (`2r`) and length in the UI.
- The **largest-radius, longest-extent** bore axis is the prime primary-axis
  candidate.

### 5. Reflective symmetry plane
Triple clamps are nominally bilaterally symmetric — a strong orientation cue and a
natural source for the Right datum / center origin.

- Global PCA of the vertices; candidate symmetry-plane normals = the 3 principal
  axes through the centroid.
- Score each: reflect a sample of vertices across the candidate plane, query nearest
  original vertex via nanoflann, accumulate distance. Symmetry score = inverse mean
  nearest-neighbor distance (normalized by bbox). Lowest mismatch wins.
- **Refine (stretch):** point-to-plane ICP between the mesh and its reflection; the
  symmetry plane is the bisector of the resulting rigid alignment.

### 6. Assigning Top / Front / Right + origin
Heuristic and part-dependent — fully user-overridable. Default strategy:

- **Primary axis** by cue priority: (1) dominant bore axis, (2) symmetry-plane
  normal, (3) largest planar-face normal.
- **Convention:** largest flat face normal → **Top (Z)** (part sits on a bed);
  symmetry-plane normal → **Right (X)**; **Front (Y) = Z × X**, then
  re-orthogonalize X. Build the rotation as columns `[X Y Z]`; force right-handed
  (`det = +1`) with a deterministic sign convention (Z away from material side; X
  toward a chosen reference). For a triple clamp the steerer bore is typically
  vertical, so "bore-axis → Z" is a selectable rule.
- **Origin / datum:** intersection of three orthogonal datum planes if available;
  else the primary bore axis projected onto the Top plane; else
  symmetry-plane ∩ top-plane ∩ centroid. Provide a default and let the user nudge.
- **Rotation X/Y/Z display:** `mat.eulerAngles(0,1,2)` (rad → deg). Euler is
  ambiguous — the full matrix is truth; Euler is readout only.
- **Overall confidence:** weighted blend of the confidences of the cues actually
  used, penalized by orthogonality error (deviation from 90° before forced
  orthonormalization) and by the winner-vs-runner-up margin. `method` records the
  winning cue ("bore-axis + symmetry", "3 orthogonal planes", "PCA-fallback").
- **Fallback:** if nothing scores well, orient by global PCA (principal axes →
  X/Y/Z by descending eigenvalue), low confidence, method "PCA-fallback".

### 7. Classification + shape readout
- Each chosen datum plane is tagged with a **tier** (Top→Primary, Right→Tertiary)
  and a **source** describing how it was derived (`face`, `symmetry`, `bore`);
  everything else is `Candidate · feature`.
- A candidate flat is reclassified **`bore-seat`** when its normal is parallel to a
  detected cylinder axis (`|n·axis| > 0.9`) and its centroid lies within ~2× the
  bore radius of the axis (a flat seat ringing a bore).
- The part's **PCA elongation ratio** (vertex covariance eigenvalues, largest:mid:1)
  is stored on `OrientationResult.pcaRatio` purely as a shape readout.
- We only emit labels we can actually derive — no fabricated `midpoint`/`bbox`
  metadata.

---

## Deviation / heatmap

Two modes (both requested).

### Mode a — best-fit vs a loaded reference mesh
- **Coarse init:** align centroids + principal axes (PCA) of scan vs reference to
  avoid local minima; optionally start ICP from current orientation.
- **Point-to-plane ICP** (faster/better convergence on smooth CAD): for each scan
  vertex find the closest point on the reference surface via our `ma::Bvh`
  (median-split AABB tree, closest-point-on-triangle query); use the reference face
  normal; minimize Σ((R·pᵢ + t − qᵢ)·nᵢ)². Linearize rotation (small-angle) → 6×6
  linear system solved with Eigen `ldlt`. Iterate to convergence. Robustify with
  pair rejection (distance > 3·median). PCA coarse init (with sign-flip search)
  seeds the rotation. `ma::icpToReference`.

### Mode b — vs the part's own datums / symmetry
- `deviationToPlane`: signed distance of each scan vertex to a fitted datum plane
  (Top, else symmetry). No reference model needed.

### Signed distance
- Per scan vertex: closest point on the reference via `ma::Bvh`; **sign** from the
  reference face normal (`sign((p−q)·n)`). Per-vertex signed distance in mm.
  Convention: **positive = material excess** (outside the reference).
- **Per-datum flatness:** `ma::planeResidual` returns RMS / max distance of a
  detected plane's own inlier vertices to its fitted plane — shown in the per-datum
  editor.

### Tolerance banding + color mapping
- `ToleranceBands { nominal = 0, inTolMm, warnMm }`. Coloring (`bandColor`):
  - `|d| ≤ inTol` → **green** (in-tolerance overlay)
  - else a **diverging ramp** white(0) → red(+, proud) / blue(−, recessed) across
    `[−range, range]`.
- Implementation: upload the per-vertex signed distance as a **vertex scalar
  attribute once**; color via a 1D transfer-function (LUT) texture sampled in the
  fragment shader. Changing the tolerance or range swaps the LUT live — no geometry
  re-upload.
- **Range** is auto (data-driven `max|d|`) or a manual ± value (Auto-range toggle).
- **Scalar bar:** an ImGui overlay (gradient rect + mm tick labels) reading the same
  `bandColor`, so legend and shading never disagree. Overall RMS + % in-tolerance
  are reported alongside.

---

## Picking (manual datum tools)

- Build the pick ray from **viewport-local** mouse coords (relative to the
  `ImGui::Image` sub-rect, not the window), unproject near/far via the inverse
  view-proj.
- Map the world ray into part space with `previewTransform().inverse()`, then
  `ma::raycast` (Möller–Trumbore over faces) → hit face + position.
- **3-point plane:** collect 3 hits; normal `(p2−p1)×(p3−p1)`, point p1; snap normal
  toward the camera; assign to the chosen datum via `assignDatum`.
- **Hole / cylinder pick:** ray-hit a face on the bore; `growSmoothRegion` floods
  across adjacency while the dihedral angle stays small (captures the curved wall,
  stops at sharp edges); run the §4 cylinder fit on just that local set; the fitted
  cylinder is added to the detected list and its axis assigned to a datum.
- Picked points render as billboarded markers (PlaneRenderer) over the scene.

### Per-datum adjustment (Apply Adjustment)
- Expanding a datum row exposes a live editor: rotate about a chosen datum axis
  (through the origin) by an angle + offset. The preview composes
  `pendingDelta * committedTransform` into `previewTransform()` (what the viewport
  and picking use); **Apply** bakes it into the user transform, **Reset** discards.

---

## Sources / references
- Möller–Trumbore ray–triangle intersection; Ericson, *Real-Time Collision
  Detection* (closest-point-on-triangle, AABB queries).
- Efficient RANSAC for point-clouds (Schnabel et al.) — the plane/cylinder
  detection basis.
- Point-to-plane ICP (Chen & Medioni) — the deviation best-fit solver.
- Point-to-plane ICP (Chen & Medioni) — the deviation/best-fit solver.
