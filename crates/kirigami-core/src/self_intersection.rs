use crate::render::RenderSnapshot;
use crate::topology::VertexId;
use crate::{FoldRequest, ModelError, PanelId, PaperModel};
use bvh_kernels::StaticBvh;
use geometry_kernels::{
    gjk::{GjkStatus, gjk_intersection},
    support::ConvexHull3,
};
use serde::Serialize;
use spatial_kernels::{Aabb, Body};
use std::collections::{HashMap, HashSet};

const CONTACT_INSET_RATIO: f64 = 1.0e-6;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SelfIntersectionScope {
    /// Tests every distinct panel pair. Pairs that intentionally share a seam or
    /// topology vertex are rechecked after a tiny deterministic interior inset so
    /// boundary-only contact stays legal while interior overlap is still reported.
    AllPanelPairsContactAware,
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
}

impl PaperModel {
    /// Reports BVH-pruned intersections between distinct paper panels.
    /// Intentional seam/vertex contact is filtered from true interior overlap.
    /// The query is advisory: it does not yet reject an otherwise-valid fold.
    pub fn self_intersection_report(
        &self,
        fold: Option<FoldRequest>,
    ) -> Result<SelfIntersectionReport, ModelError> {
        match fold {
            Some(request) => self.self_intersection_report_with_folds(&[request]),
            None => self.self_intersection_report_with_folds(&[]),
        }
    }

    pub fn self_intersection_report_with_folds(
        &self,
        folds: &[FoldRequest],
    ) -> Result<SelfIntersectionReport, ModelError> {
        let snapshot = self.render_snapshot_with_folds(folds)?;
        let intentional_contacts = self.intentional_contact_pairs()?;
        analyze_self_intersections(&snapshot, &intentional_contacts)
    }

    pub(super) fn intentional_contact_pairs(
        &self,
    ) -> Result<HashSet<(PanelId, PanelId)>, ModelError> {
        let mut contacts: HashSet<(PanelId, PanelId)> = self
            .seams
            .iter()
            .filter(|seam| seam.panel_a != seam.panel_b)
            .map(|seam| canonical_panel_pair(seam.panel_a, seam.panel_b))
            .collect();
        let mut panels_at_vertex = HashMap::<VertexId, Vec<PanelId>>::new();
        for panel in &self.panels {
            for vertex in self.topology.face_vertex_ids(panel.face)? {
                panels_at_vertex.entry(vertex).or_default().push(panel.id);
            }
        }
        for panels in panels_at_vertex.values() {
            for (index, left) in panels.iter().enumerate() {
                for right in &panels[index + 1..] {
                    contacts.insert(canonical_panel_pair(*left, *right));
                }
            }
        }
        Ok(contacts)
    }
}

pub(super) fn analyze_self_intersections(
    snapshot: &RenderSnapshot,
    intentional_contacts: &HashSet<(PanelId, PanelId)>,
) -> Result<SelfIntersectionReport, ModelError> {
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
        bodies.push(Body::new(
            collision_triangle_id(triangle_index)?,
            Aabb::new(min, max),
        ));
    }

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
        if left_panel == right_panel {
            continue;
        }

        let left_vertices =
            triangle_vertices(snapshot, left_index).map(|vertex| vertex.map(f64::from));
        let right_vertices =
            triangle_vertices(snapshot, right_index).map(|vertex| vertex.map(f64::from));

        narrow_phase_tests += 1;
        let primary = gjk_intersection(
            &ConvexHull3::new(&left_vertices),
            &ConvexHull3::new(&right_vertices),
        )
        .status;

        let pair_has_intentional_contact =
            intentional_contacts.contains(&canonical_panel_pair(left_panel, right_panel));
        let status = if pair_has_intentional_contact && primary != GjkStatus::Separated {
            // Adjacent panels legitimately meet on their authoritative seam or shared
            // topology vertex. Insetting both convex triangles removes boundary-only
            // contact while preserving any meaningful interior overlap/penetration.
            let left_inset = inset_triangle(left_vertices);
            let right_inset = inset_triangle(right_vertices);
            narrow_phase_tests += 1;
            gjk_intersection(
                &ConvexHull3::new(&left_inset),
                &ConvexHull3::new(&right_inset),
            )
            .status
        } else {
            primary
        };

        let pair = SelfIntersectionPair {
            triangle_a: candidate.a,
            triangle_b: candidate.b,
            panel_a: left_panel,
            panel_b: right_panel,
        };
        match status {
            GjkStatus::Intersecting => intersections.push(pair),
            GjkStatus::Separated => {}
            GjkStatus::NoProgress | GjkStatus::IterationLimit => indeterminate_pairs.push(pair),
        }
    }

    Ok(SelfIntersectionReport {
        scope: SelfIntersectionScope::AllPanelPairsContactAware,
        triangle_count,
        broad_phase_candidates,
        narrow_phase_tests,
        indeterminate_pairs,
        intersections,
    })
}

fn inset_triangle(vertices: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let centroid: [f64; 3] = std::array::from_fn(|axis| {
        (vertices[0][axis] + vertices[1][axis] + vertices[2][axis]) / 3.0
    });
    std::array::from_fn(|vertex_index| {
        std::array::from_fn(|axis| {
            centroid[axis]
                + (vertices[vertex_index][axis] - centroid[axis]) * (1.0 - CONTACT_INSET_RATIO)
        })
    })
}

pub(super) fn collision_triangle_id(index: usize) -> Result<u32, ModelError> {
    u32::try_from(index).map_err(|_| ModelError::TooManyCollisionTriangles)
}

fn triangle_vertices(snapshot: &RenderSnapshot, triangle_index: usize) -> [[f32; 3]; 3] {
    let offset = triangle_index * 3;
    std::array::from_fn(|corner| snapshot.vertices[snapshot.indices[offset + corner] as usize])
}

pub(super) fn canonical_panel_pair(left: PanelId, right: PanelId) -> (PanelId, PanelId) {
    if left.0 <= right.0 {
        (left, right)
    } else {
        (right, left)
    }
}
