# Kirigami roadmap

## Authority

`kirigami-core` is authoritative for paper topology, cuts, creases, fold constraints, connectivity, and validity. Renderers, editors, physics, and browser code consume snapshots or adapters and must not redefine those semantics.

`PlanarTopology` inside `kirigami-core` is authoritative for 2D geometric subdivision. Cut versus crease semantics remain separate from geometric adjacency, so topology can be reused without making the renderer or a generic geometry layer authoritative for paper behavior.

`three-d-core` is the renderer-neutral mesh boundary. Kirigami consumes its pinned geometry vocabulary instead of copying those primitives locally.

## Implemented foundation

- Rectangular sheet creation.
- Stable panel, logical operation, and seam-segment identities.
- Half-edge planar topology with stable faces, reciprocal twins, and invariant validation.
- Transactional face splitting by a straight boundary-to-boundary segment.
- Shared-edge subdivision when a later operation terminates on an existing crease.
- Rejection of chords that leave/cross a simple face.
- Deterministic ear-clipping triangulation for simple concave faces; rendering no longer assumes convex fan triangulation.
- Transactional single-face polyline splitting for bent cuts and segmented straight creases.
- Deterministic multi-face path tracing with inserted boundary intersections; one logical operation spans all generated seam segments.
- Closed cuts that create detached inner panels plus authoritative outer/inner face boundary loops.
- Multiple disjoint or nested hole loops with deterministic hole-aware rendering triangulation.
- Boundary-bridge cuts that connect outer-to-hole or hole-to-hole components without inventing a new panel.
- BVH-pruned, GJK-tested advisory self-intersection reports across all distinct panel pairs; seam/shared-vertex neighbors use a deterministic interior-inset recheck so legal boundary contact is ignored while adjacent-panel overlap remains visible, with indeterminate results fail-closed.
- Cuts that split connected components.
- Creases that preserve connectivity.
- Deterministic rigid rotation around a crease axis.
- Deterministic multi-crease fold states for straight creases, with canonical operation ordering, world-space hinge propagation, and fail-closed detection when an earlier fold kinks a later rigid crease.
- Renderer-neutral immutable snapshots plus `three-d-core::Mesh` conversion.
- Authoritative flat-pattern snapshots plus deterministic print-ready PDF, physical-size SVG, and FOLD 1.2 crease-pattern export; FOLD uses subdivided topology segments, `B` boundaries, `C` cuts, and `U` creases until mountain/valley assignments become authoritative.
- Geometry-only local flat-foldability reports for interior crease vertices, checking even crease degree and Kawasaki's alternating-angle condition while excluding external/cut boundary vertices.
- Rust/WASM browser proof and GitHub Pages deployment.
- Browser upload for OBJ/glTF/GLB mesh targets and JSON skeleton targets, normalized outside `kirigami-core` through pinned `3d-lab` format, mesh, and animation contracts.
- Deterministic target centering/uniform scaling to unit max extent, exact-source FNV-1a fingerprints, normalization receipts, normalized mesh/skeleton measurements, and a public integration test suite covering OBJ, glTF, GLB, skeletons, equivalence invariants, and fail-closed invalid inputs.

## Next vertical slices

1. Upgrade rendering/physics triangulation to constrained Delaunay when mesh quality requires it; the authoritative boundary graph remains independent of the triangulator.
2. Extend the fold constraint solver from rigid straight multi-crease states to crossing and genuinely bent creases. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
3. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
4. Extend flat-foldability validation from local geometry checks to mountain/valley assignments, global layer ordering, and global validity.
5. Add FOLD import and deterministic pattern fixtures; PDF, SVG, and FOLD export are implemented.
6. Add the 3D-target approximation layer described below once forward folding, collisions, and validity can score candidates reliably.
7. Add broader inverse-design/optimization experiments only after the target-approximation loop is deterministic and measurable.

## Algorithm priorities

Closed-path/hole arrangement, multiple boundary loops, and boundary bridges are part of the deterministic topology foundation. Self-intersection reuses the pinned `rust-kernels` static BVH for deterministic candidate pruning and generic GJK convex-hull queries for triangle tests. Adjacent panels are now included: a second deterministic GJK pass over slightly inset triangles removes legal boundary-only contact without hiding interior overlap. The next algorithm work is constrained triangulation and extending the deterministic fold solver to crossing/bent crease constraints.


## 3D target approximation

The browser can now accept an uploaded OBJ/glTF/GLB mesh or a simple JSON skeleton. File-format ingestion is not a `kirigami-core` responsibility: `kirigami-targets` delegates mesh decoding to pinned `3d-lab` format/asset contracts and skeleton hierarchy evaluation to `three-d-animation`, then exposes one renderer-neutral target snapshot. A loaded target is still only input evidence; it does not redefine paper topology or validity.

The approximation layer should remain outside the authoritative paper model. It proposes deterministic candidate command sequences (panels, cuts, creases, and fold targets); `kirigami-core` validates and evaluates those candidates using the same topology and folding rules as hand-authored patterns. The initial objective should balance surface/shape error against panel count, total cut length, crease complexity, fold-angle complexity, self-intersection, flat-foldability/manufacturability, and material bounds. Candidate generation must retain seeds, input fingerprints, objective weights, and evaluation receipts so improvements can be benchmarked over time.

Uploaded targets are now centered and uniformly scaled so their largest extent is 1, with source axes intentionally preserved. The snapshot retains a deterministic exact-byte fingerprint, source bounds/scale receipt, normalized mesh surface area, and normalized skeleton edge length. The next target-specific slice should make orientation policy explicit and add the first candidate-to-target scorer: skeleton correspondence first, then mesh surface/shape error after simplification or developable-patch segmentation.
