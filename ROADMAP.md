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
- Cuts that split connected components.
- Creases that preserve connectivity.
- Deterministic rigid rotation around a crease axis.
- Renderer-neutral immutable snapshots plus `three-d-core::Mesh` conversion.
- Rust/WASM browser proof and GitHub Pages deployment.

## Next vertical slices

1. Add closed cut paths / holes and multiple boundary loops; then upgrade triangulation to constrained Delaunay when mesh quality or physical simulation requires it while keeping boundary constraints authoritative.
2. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
3. Add multi-crease fold state and constraint propagation so genuinely bent creases have an authoritative solver. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
4. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
5. Add layer ordering and flat-foldability validation.
6. Add SVG/FOLD import/export and deterministic pattern fixtures.
7. Add inverse-design/optimization experiments only after forward topology and validity are well tested.

## Algorithm priorities

The immediate algorithm work is closed-path/hole arrangement, constrained triangulation with multiple boundary loops, BVH/self-intersection, and then multi-crease constraint solving plus layer ordering. Multi-face open-path intersection insertion is now part of the deterministic topology foundation.
