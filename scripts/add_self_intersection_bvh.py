from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise SystemExit(f"{label}: pattern not found")
    return text.replace(old, new, 1)


RUST_KERNELS_REV = "980091094a9826167a98b96781126595fbea5076"

# Workspace dependencies -----------------------------------------------------------
root = Path("Cargo.toml")
text = root.read_text()
text = replace_once(
    text,
    '''earcut = "0.4.11"
wasm-bindgen = "0.2"
three-d-core = { git = "https://github.com/moritzbrantner/3d-lab", rev = "419333a7ecedc94a99bfc6b940fbdbcfc7bec958", package = "three-d-core" }''',
    f'''earcut = "0.4.11"
wasm-bindgen = "0.2"
bvh-kernels = {{ git = "https://github.com/moritzbrantner/rust-kernels", rev = "{RUST_KERNELS_REV}" }}
geometry-kernels = {{ git = "https://github.com/moritzbrantner/rust-kernels", rev = "{RUST_KERNELS_REV}" }}
spatial-kernels = {{ git = "https://github.com/moritzbrantner/rust-kernels", rev = "{RUST_KERNELS_REV}" }}
three-d-core = {{ git = "https://github.com/moritzbrantner/3d-lab", rev = "419333a7ecedc94a99bfc6b940fbdbcfc7bec958", package = "three-d-core" }}''',
    "workspace collision dependencies",
)
root.write_text(text)

core_manifest = Path("crates/kirigami-core/Cargo.toml")
text = core_manifest.read_text()
text = replace_once(
    text,
    '''[dependencies]
earcut.workspace = true
serde.workspace = true
three-d-core.workspace = true''',
    '''[dependencies]
bvh-kernels.workspace = true
earcut.workspace = true
geometry-kernels.workspace = true
serde.workspace = true
spatial-kernels.workspace = true
three-d-core.workspace = true''',
    "core collision dependencies",
)
core_manifest.write_text(text)

# Core API ------------------------------------------------------------------------
lib_path = Path("crates/kirigami-core/src/lib.rs")
lib = lib_path.read_text()

lib = replace_once(
    lib,
    '''use serde::Serialize;
use std::collections::{HashSet, VecDeque};
use std::fmt;
use three_d_core::{Mesh, MeshError, Vec3};''',
    '''use bvh_kernels::StaticBvh;
use geometry_kernels::{
    gjk::{GjkStatus, gjk_intersection},
    support::ConvexHull3,
};
use serde::Serialize;
use spatial_kernels::{Aabb, Body};
use std::collections::{HashSet, VecDeque};
use std::fmt;
use three_d_core::{Mesh, MeshError, Vec3};''',
    "collision imports",
)

lib = replace_once(
    lib,
    '''pub struct RenderSnapshot {
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    pub seams: Vec<RenderSeam>,
    pub panel_count: usize,
    pub component_count: usize,
}''',
    '''pub struct RenderSnapshot {
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    /// One panel owner per triangle, in the same order as `indices.chunks_exact(3)`.
    pub triangle_panels: Vec<PanelId>,
    pub seams: Vec<RenderSeam>,
    pub panel_count: usize,
    pub component_count: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SelfIntersectionScope {
    /// Tests only panel pairs that do not already share any cut or crease seam.
    /// This avoids reporting intended hinge/cut-boundary contact as penetration.
    NonNeighborPanels,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct SelfIntersectionPair {
    pub triangle_a: u32,
    pub triangle_b: u32,
    pub panel_a: PanelId,
    pub panel_b: PanelId,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SelfIntersectionReport {
    pub scope: SelfIntersectionScope,
    pub triangle_count: usize,
    pub broad_phase_candidates: usize,
    pub narrow_phase_tests: usize,
    /// GJK pairs whose bounded deterministic query could not decide intersection.
    /// Callers must not treat an indeterminate report as proven collision-free.
    pub indeterminate_pairs: Vec<SelfIntersectionPair>,
    pub intersections: Vec<SelfIntersectionPair>,
}

impl SelfIntersectionReport {
    #[must_use]
    pub fn is_proven_clear(&self) -> bool {
        self.intersections.is_empty() && self.indeterminate_pairs.is_empty()
    }
}''',
    "self intersection report structs",
)

lib = replace_once(
    lib,
    '''        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        for panel in &self.panels {''',
    '''        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut triangle_panels = Vec::new();

        for panel in &self.panels {''',
    "triangle owner allocation",
)

lib = replace_once(
    lib,
    '''            for triangle in triangulation.triangles {
                for local_index in triangle {
                    let global_index = panel_start
                        .checked_add(local_index as usize)
                        .ok_or(ModelError::TooManyRenderVertices)?;
                    indices.push(
                        u32::try_from(global_index)
                            .map_err(|_| ModelError::TooManyRenderVertices)?,
                    );
                }
            }
        }''',
    '''            for triangle in triangulation.triangles {
                for local_index in triangle {
                    let global_index = panel_start
                        .checked_add(local_index as usize)
                        .ok_or(ModelError::TooManyRenderVertices)?;
                    indices.push(
                        u32::try_from(global_index)
                            .map_err(|_| ModelError::TooManyRenderVertices)?,
                    );
                }
                triangle_panels.push(panel.id);
            }
        }''',
    "triangle owner recording",
)

lib = replace_once(
    lib,
    '''        Ok(RenderSnapshot {
            vertices,
            indices,
            seams,
            panel_count: self.panels.len(),
            component_count: self.component_count(),
        })
    }

    pub fn three_d_mesh''',
    '''        Ok(RenderSnapshot {
            vertices,
            indices,
            triangle_panels,
            seams,
            panel_count: self.panels.len(),
            component_count: self.component_count(),
        })
    }

    /// Reports BVH-pruned intersections between non-neighbor paper panels.
    /// The query is advisory: it does not yet reject an otherwise-valid fold.
    pub fn self_intersection_report(
        &self,
        fold: Option<FoldRequest>,
    ) -> Result<SelfIntersectionReport, ModelError> {
        let snapshot = self.render_snapshot(fold)?;
        Ok(analyze_self_intersections(&snapshot, &self.seams))
    }

    pub fn three_d_mesh''',
    "public self intersection method",
)

analyzer = r'''
fn analyze_self_intersections(snapshot: &RenderSnapshot, seams: &[Seam]) -> SelfIntersectionReport {
    debug_assert_eq!(snapshot.indices.len() / 3, snapshot.triangle_panels.len());
    let triangle_count = snapshot.triangle_panels.len();
    let mut bodies = Vec::with_capacity(triangle_count);
    for triangle_index in 0..triangle_count {
        let vertices = triangle_vertices(snapshot, triangle_index);
        let min = std::array::from_fn(|axis| {
            vertices
                .iter()
                .map(|vertex| vertex[axis])
                .fold(f32::INFINITY, f32::min)
        });
        let max = std::array::from_fn(|axis| {
            vertices
                .iter()
                .map(|vertex| vertex[axis])
                .fold(f32::NEG_INFINITY, f32::max)
        });
        bodies.push(Body::new(triangle_index as u32, Aabb::new(min, max)));
    }

    let neighbors: HashSet<(PanelId, PanelId)> = seams
        .iter()
        .filter(|seam| seam.panel_a != seam.panel_b)
        .map(|seam| canonical_panel_pair(seam.panel_a, seam.panel_b))
        .collect();
    let bvh = StaticBvh::build(&bodies);
    let raw_candidates = bvh.overlapping_pairs();
    let broad_phase_candidates = raw_candidates.len();
    let mut narrow_phase_tests = 0;
    let mut intersections = Vec::new();
    let mut indeterminate_pairs = Vec::new();

    for candidate in raw_candidates {
        let left_index = candidate.a as usize;
        let right_index = candidate.b as usize;
        let left_panel = snapshot.triangle_panels[left_index];
        let right_panel = snapshot.triangle_panels[right_index];
        if left_panel == right_panel
            || neighbors.contains(&canonical_panel_pair(left_panel, right_panel))
        {
            continue;
        }

        narrow_phase_tests += 1;
        let left_vertices = triangle_vertices(snapshot, left_index).map(|vertex| vertex.map(f64::from));
        let right_vertices =
            triangle_vertices(snapshot, right_index).map(|vertex| vertex.map(f64::from));
        let result = gjk_intersection(
            &ConvexHull3::new(&left_vertices),
            &ConvexHull3::new(&right_vertices),
        );
        let pair = SelfIntersectionPair {
            triangle_a: candidate.a,
            triangle_b: candidate.b,
            panel_a: left_panel,
            panel_b: right_panel,
        };
        match result.status {
            GjkStatus::Intersecting => intersections.push(pair),
            GjkStatus::Separated => {}
            GjkStatus::NoProgress | GjkStatus::IterationLimit => indeterminate_pairs.push(pair),
        }
    }

    SelfIntersectionReport {
        scope: SelfIntersectionScope::NonNeighborPanels,
        triangle_count,
        broad_phase_candidates,
        narrow_phase_tests,
        indeterminate_pairs,
        intersections,
    }
}

fn triangle_vertices(snapshot: &RenderSnapshot, triangle_index: usize) -> [[f32; 3]; 3] {
    let offset = triangle_index * 3;
    std::array::from_fn(|corner| {
        snapshot.vertices[snapshot.indices[offset + corner] as usize]
    })
}

fn canonical_panel_pair(left: PanelId, right: PanelId) -> (PanelId, PanelId) {
    if left.0 <= right.0 {
        (left, right)
    } else {
        (right, left)
    }
}

'''
if "fn analyze_self_intersections" not in lib:
    marker = "fn validate_operation_path(path: &[Point2], kind: OperationKind) -> Result<(), ModelError> {"
    if marker not in lib:
        raise SystemExit("self intersection analyzer marker not found")
    lib = lib.replace(marker, analyzer + marker, 1)

if "fn bvh_self_intersection_finds_non_neighbor_crossing" not in lib:
    marker = '''    #[test]
    fn boundary_bridge_cut_opens_annulus_without_new_panel() {'''
    tests = r'''    #[test]
    fn bvh_self_intersection_finds_non_neighbor_crossing() {
        let snapshot = RenderSnapshot {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.25, 0.25, -1.0],
                [0.25, 0.25, 1.0],
                [0.75, 0.25, 0.0],
            ],
            indices: vec![0, 1, 2, 3, 4, 5],
            triangle_panels: vec![PanelId(0), PanelId(2)],
            seams: Vec::new(),
            panel_count: 2,
            component_count: 2,
        };
        let report = analyze_self_intersections(&snapshot, &[]);
        assert_eq!(report.triangle_count, 2);
        assert_eq!(report.narrow_phase_tests, 1);
        assert_eq!(report.intersections.len(), 1);
        assert!(report.indeterminate_pairs.is_empty());
        assert!(!report.is_proven_clear());
    }

    #[test]
    fn self_intersection_scope_ignores_intended_neighbor_contact() {
        let snapshot = RenderSnapshot {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, -1.0, 0.0],
            ],
            indices: vec![0, 1, 2, 3, 4, 5],
            triangle_panels: vec![PanelId(0), PanelId(1)],
            seams: Vec::new(),
            panel_count: 2,
            component_count: 1,
        };
        let neighbor = Seam {
            id: SeamId(0),
            operation: OperationId(0),
            kind: OperationKind::Crease,
            start: Point2::new(0.0, 0.0),
            end: Point2::new(1.0, 0.0),
            panel_a: PanelId(0),
            panel_b: PanelId(1),
        };
        let report = analyze_self_intersections(&snapshot, &[neighbor]);
        assert!(report.is_proven_clear());
        assert_eq!(report.narrow_phase_tests, 0);
    }

    #[test]
    fn paper_model_self_intersection_query_is_clear_at_rest() {
        let mut model = PaperModel::rectangle(4.0, 2.0).unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
                OperationKind::Crease,
            )
            .unwrap();
        let report = model.self_intersection_report(None).unwrap();
        assert!(report.is_proven_clear());
        assert!(report.intersections.is_empty());
    }

    #[test]
    fn boundary_bridge_cut_opens_annulus_without_new_panel() {'''
    if marker not in lib:
        raise SystemExit("self intersection test marker not found")
    lib = lib.replace(marker, tests, 1)

lib_path.write_text(lib)

# Roadmap -------------------------------------------------------------------------
roadmap_path = Path("ROADMAP.md")
roadmap = roadmap_path.read_text()
roadmap = replace_once(
    roadmap,
    '''- Boundary-bridge cuts that connect outer-to-hole or hole-to-hole components without inventing a new panel.
- Cuts that split connected components.''',
    '''- Boundary-bridge cuts that connect outer-to-hole or hole-to-hole components without inventing a new panel.
- BVH-pruned, GJK-tested advisory self-intersection reports for non-neighbor panel pairs, with indeterminate pairs surfaced fail-closed.
- Cuts that split connected components.''',
    "roadmap collision implemented",
)
roadmap = replace_once(
    roadmap,
    '''1. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
2. Upgrade rendering/physics triangulation to constrained Delaunay when mesh quality requires it; the authoritative boundary graph remains independent of the triangulator.
3. Add multi-crease fold state and constraint propagation so genuinely bent creases have an authoritative solver. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
4. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
5. Add layer ordering and flat-foldability validation.
6. Add SVG/FOLD import/export and deterministic pattern fixtures.
7. Add the 3D-target approximation layer described below once forward folding, collisions, and validity can score candidates reliably.
8. Add broader inverse-design/optimization experiments only after the target-approximation loop is deterministic and measurable.''',
    '''1. Extend collision validation from non-neighbor panels to adjacent-panel penetration while distinguishing intentional shared-boundary contact from overlap.
2. Upgrade rendering/physics triangulation to constrained Delaunay when mesh quality requires it; the authoritative boundary graph remains independent of the triangulator.
3. Add multi-crease fold state and constraint propagation so genuinely bent creases have an authoritative solver. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
4. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
5. Add layer ordering and flat-foldability validation.
6. Add SVG/FOLD import/export and deterministic pattern fixtures.
7. Add the 3D-target approximation layer described below once forward folding, collisions, and validity can score candidates reliably.
8. Add broader inverse-design/optimization experiments only after the target-approximation loop is deterministic and measurable.''',
    "roadmap collision next",
)
roadmap = replace_once(
    roadmap,
    '''Closed-path/hole arrangement, multiple boundary loops, and boundary bridges are now part of the deterministic topology foundation. Hole-aware rendering uses Earcut as a replaceable consumer of authoritative boundary geometry; the next algorithm work is BVH/self-intersection, then constrained triangulation and multi-crease constraint solving.''',
    '''Closed-path/hole arrangement, multiple boundary loops, and boundary bridges are part of the deterministic topology foundation. Self-intersection now reuses the pinned `rust-kernels` static BVH for deterministic candidate pruning and generic GJK convex-hull queries for triangle tests; the first accepted scope intentionally excludes topological neighbor panels. The next algorithm work is contact-aware adjacent-panel validation, constrained triangulation, and multi-crease constraint solving.''',
    "roadmap collision priorities",
)
roadmap_path.write_text(roadmap)
