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
- Cuts that split connected components.
- Creases that preserve connectivity.
- Deterministic rigid rotation around a crease axis.
- Renderer-neutral immutable snapshots plus `three-d-core::Mesh` conversion.
- Rust/WASM browser proof and GitHub Pages deployment.

## Next vertical slices

1. Generalize operations from one chord to arbitrary cut and crease polylines, including multiple face crossings and deterministic intersection insertion.
2. Add holes/multiple boundary loops and upgrade triangulation to constrained Delaunay when mesh quality or physical simulation requires it; keep boundary constraints authoritative regardless of triangulator.
3. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
4. Add multi-crease fold state and constraint propagation. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
5. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
6. Add layer ordering and flat-foldability validation.
7. Add SVG/FOLD import/export and deterministic pattern fixtures.
8. Add inverse-design/optimization experiments only after forward topology and validity are well tested.

## Algorithm priorities

The immediate algorithm work is robust segment intersection across multiple faces, polyline arrangement insertion, constrained triangulation with holes, connected-components traversal, and BVH/self-intersection. Constraint solving and layer ordering follow once multi-operation topology is stable.
