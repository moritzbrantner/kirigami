# Kirigami roadmap

## Authority

`kirigami-core` is authoritative for paper topology, cuts, creases, fold constraints, connectivity, and validity. Renderers, editors, physics, and browser code consume snapshots or adapters and must not redefine those semantics.

`three-d-core` is the renderer-neutral mesh boundary. The first slice pins the current `3d-lab` revision instead of copying its geometry primitives.

## Implemented foundation

- Rectangular sheet creation.
- Stable panel and seam identities.
- Convex panel splitting by a straight boundary-to-boundary operation.
- Cuts that split connected components.
- Creases that preserve connectivity.
- Deterministic rigid rotation around a crease axis.
- Renderer-neutral immutable snapshots plus `three-d-core::Mesh` conversion.
- Rust/WASM browser proof and GitHub Pages deployment.

## Next vertical slices

1. Replace convex fan triangulation with a reusable planar subdivision / half-edge representation and constrained triangulation.
2. Generalize operations to arbitrary cut and crease polylines, including multiple intersections and holes.
3. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
4. Add multi-crease fold state and constraint propagation. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
5. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
6. Add layer ordering and flat-foldability validation.
7. Add SVG/FOLD import/export and deterministic pattern fixtures.
8. Add inverse-design/optimization experiments only after forward topology and validity are well tested.

## Algorithm priorities

The most important reusable algorithm work is planar subdivision/half-edge topology, robust segment intersection, constrained triangulation, connected-components traversal, BVH/self-intersection, and later constraint solving plus layer ordering.
