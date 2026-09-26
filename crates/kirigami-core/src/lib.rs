//! Authoritative kirigami sheet topology and fold semantics.
//!
//! Rendering, browser interaction, and physical material simulation are consumers of
//! this crate. Cuts and creases are topology operations here, not renderer effects.

mod topology;

pub use topology::{
    FaceBoundaryLoops, FaceId, FaceTriangulation, HalfEdgeId, PlanarTopology, TopologyError,
    VertexId,
};

use bvh_kernels::StaticBvh;
use geometry_kernels::{
    gjk::{GjkStatus, gjk_intersection},
    support::ConvexHull3,
};
use serde::Serialize;
use spatial_kernels::{Aabb, Body};
use std::collections::{HashSet, VecDeque};
use std::fmt;
use three_d_core::{Mesh, MeshError, Vec3};

const EPSILON: f32 = 1.0e-5;
const CONTACT_INSET_RATIO: f64 = 1.0e-6;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}

impl Point2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum OperationKind {
    Crease,
    Cut,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct PanelId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct OperationId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct SeamId(pub u32);

#[derive(Debug, Clone, PartialEq)]
pub struct Panel {
    pub id: PanelId,
    pub face: FaceId,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Operation {
    pub id: OperationId,
    pub kind: OperationKind,
    pub path: Vec<Point2>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Seam {
    pub id: SeamId,
    pub operation: OperationId,
    pub kind: OperationKind,
    pub start: Point2,
    pub end: Point2,
    /// Creases connect two panels. A cut boundary bridge can have the same panel on
    /// both sides because it opens material without creating a second panel.
    pub panel_a: PanelId,
    pub panel_b: PanelId,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PaperModel {
    topology: PlanarTopology,
    panels: Vec<Panel>,
    operations: Vec<Operation>,
    seams: Vec<Seam>,
    next_panel_id: u32,
    next_operation_id: u32,
    next_seam_id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FoldRequest {
    pub operation: OperationId,
    pub angle_radians: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RenderSeam {
    pub operation: OperationId,
    pub kind: OperationKind,
    pub start: [f32; 3],
    pub end: [f32; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RenderSnapshot {
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ModelError {
    InvalidSheetDimensions,
    NonFinitePoint,
    DegenerateSegment,
    InvalidPath,
    BentCreaseRequiresConstraintSolver,
    DuplicateFoldOperation(OperationId),
    FoldConstraintConflict(OperationId),
    UnknownPanel(PanelId),
    UnknownOperation(OperationId),
    SegmentEndpointOffBoundary,
    SegmentDoesNotSplitPanel,
    CannotFoldCut(OperationId),
    NonFiniteFoldAngle,
    SeamReattachmentFailed(SeamId),
    Topology(TopologyError),
    TooManyRenderVertices,
    TooManyCollisionTriangles,
}

impl fmt::Display for ModelError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSheetDimensions => {
                formatter.write_str("sheet dimensions must be finite and positive")
            }
            Self::NonFinitePoint => {
                formatter.write_str("segment points must contain only finite coordinates")
            }
            Self::DegenerateSegment => formatter.write_str("segment endpoints must be distinct"),
            Self::InvalidPath => formatter.write_str("a path requires at least two finite points"),
            Self::BentCreaseRequiresConstraintSolver => formatter
                .write_str("a non-straight crease requires the multi-crease constraint solver"),
            Self::DuplicateFoldOperation(operation) => write!(
                formatter,
                "fold operation {} was requested more than once",
                operation.0
            ),
            Self::FoldConstraintConflict(operation) => write!(
                formatter,
                "fold operation {} cannot be satisfied by the current rigid crease state",
                operation.0
            ),
            Self::UnknownPanel(panel) => write!(formatter, "unknown panel {}", panel.0),
            Self::UnknownOperation(operation) => {
                write!(formatter, "unknown operation {}", operation.0)
            }
            Self::SegmentEndpointOffBoundary => formatter
                .write_str("both segment endpoints must lie on the selected panel boundary"),
            Self::SegmentDoesNotSplitPanel => formatter
                .write_str("segment does not split the selected panel into two valid faces"),
            Self::CannotFoldCut(operation) => write!(
                formatter,
                "operation {} is a cut and cannot be folded",
                operation.0
            ),
            Self::NonFiniteFoldAngle => formatter.write_str("fold angle must be finite"),
            Self::SeamReattachmentFailed(seam) => write!(
                formatter,
                "could not reattach seam {} after splitting its incident panel",
                seam.0
            ),
            Self::Topology(error) => write!(formatter, "topology error: {error}"),
            Self::TooManyRenderVertices => {
                formatter.write_str("render snapshot exceeds u32 index capacity")
            }
            Self::TooManyCollisionTriangles => {
                formatter.write_str("collision query exceeds u32 triangle-ID capacity")
            }
        }
    }
}

impl std::error::Error for ModelError {}

impl From<TopologyError> for ModelError {
    fn from(error: TopologyError) -> Self {
        Self::Topology(error)
    }
}

impl PaperModel {
    pub fn rectangle(width: f32, height: f32) -> Result<Self, ModelError> {
        if !width.is_finite() || !height.is_finite() || width <= 0.0 || height <= 0.0 {
            return Err(ModelError::InvalidSheetDimensions);
        }
        let half_width = width * 0.5;
        let half_height = height * 0.5;
        let topology = PlanarTopology::from_polygon(vec![
            Point2::new(-half_width, -half_height),
            Point2::new(half_width, -half_height),
            Point2::new(half_width, half_height),
            Point2::new(-half_width, half_height),
        ])?;
        Ok(Self {
            topology,
            panels: vec![Panel {
                id: PanelId(0),
                face: FaceId(0),
            }],
            operations: Vec::new(),
            seams: Vec::new(),
            next_panel_id: 1,
            next_operation_id: 0,
            next_seam_id: 0,
        })
    }

    pub fn topology(&self) -> &PlanarTopology {
        &self.topology
    }

    pub fn panels(&self) -> &[Panel] {
        &self.panels
    }

    pub fn operations(&self) -> &[Operation] {
        &self.operations
    }

    pub fn seams(&self) -> &[Seam] {
        &self.seams
    }

    pub fn component_count(&self) -> usize {
        if self.panels.is_empty() {
            return 0;
        }
        let mut visited = HashSet::new();
        let mut count = 0;
        for panel in &self.panels {
            if visited.contains(&panel.id) {
                continue;
            }
            count += 1;
            let mut queue = VecDeque::from([panel.id]);
            visited.insert(panel.id);
            while let Some(current) = queue.pop_front() {
                for seam in self
                    .seams
                    .iter()
                    .filter(|seam| seam.kind == OperationKind::Crease)
                {
                    let neighbor = adjacent_panel(seam, current);
                    match neighbor {
                        Some(neighbor) if visited.insert(neighbor) => {
                            queue.push_back(neighbor);
                        }
                        _ => {}
                    }
                }
            }
        }
        count
    }

    /// Splits a panel using a straight boundary-to-boundary segment.
    pub fn split_panel_with_segment(
        &mut self,
        panel_id: PanelId,
        start: Point2,
        end: Point2,
        kind: OperationKind,
    ) -> Result<OperationId, ModelError> {
        self.split_panel_with_path(panel_id, &[start, end], kind)
    }

    /// Splits a panel along an open polyline. Bent paths are currently supported for
    /// cuts; creases must remain straight until the constraint solver owns non-rigid folds.
    pub fn split_panel_with_polyline(
        &mut self,
        panel_id: PanelId,
        path: &[Point2],
        kind: OperationKind,
    ) -> Result<OperationId, ModelError> {
        if path.len() < 2 || path.iter().any(|point| !point.is_finite()) {
            return Err(ModelError::InvalidPath);
        }
        if kind == OperationKind::Crease && !path_is_straight(path) {
            return Err(ModelError::BentCreaseRequiresConstraintSolver);
        }
        self.split_panel_with_path(panel_id, path, kind)
    }

    /// Cuts a simple closed path fully inside one panel. The enclosed material becomes
    /// a detached panel while the original panel keeps the new inner boundary.
    pub fn cut_closed_path(
        &mut self,
        panel_id: PanelId,
        path: &[Point2],
    ) -> Result<OperationId, ModelError> {
        let canonical_path = canonical_closed_path(path)?;
        let mut candidate = self.clone();
        let panel_index = candidate
            .panels
            .iter()
            .position(|panel| panel.id == panel_id)
            .ok_or(ModelError::UnknownPanel(panel_id))?;
        let original_id = candidate.panels[panel_index].id;
        let original_face = candidate.panels[panel_index].face;
        let new_face = candidate
            .topology
            .insert_closed_loop(original_face, &canonical_path)
            .map_err(map_split_error)?;
        let new_id = PanelId(candidate.next_panel_id);
        let operation_id = OperationId(candidate.next_operation_id);

        candidate.reattach_seams_inside_closed_cut(original_id, new_id, &canonical_path);
        for segment in canonical_path.windows(2) {
            candidate.seams.push(Seam {
                id: SeamId(candidate.next_seam_id),
                operation: operation_id,
                kind: OperationKind::Cut,
                start: segment[0],
                end: segment[1],
                panel_a: original_id,
                panel_b: new_id,
            });
            candidate.next_seam_id += 1;
        }
        candidate.panels.push(Panel {
            id: new_id,
            face: new_face,
        });
        candidate.operations.push(Operation {
            id: operation_id,
            kind: OperationKind::Cut,
            path: canonical_path,
        });
        candidate.next_panel_id += 1;
        candidate.next_operation_id += 1;
        *self = candidate;
        Ok(operation_id)
    }

    /// Cuts an open path between two distinct boundary components of one panel.
    /// The material remains one panel; the operation opens an annulus or merges holes.
    pub fn cut_boundary_bridge(
        &mut self,
        panel_id: PanelId,
        path: &[Point2],
    ) -> Result<OperationId, ModelError> {
        validate_operation_path(path, OperationKind::Cut)?;
        let mut candidate = self.clone();
        let panel = candidate
            .panels
            .iter()
            .find(|panel| panel.id == panel_id)
            .cloned()
            .ok_or(ModelError::UnknownPanel(panel_id))?;
        candidate
            .topology
            .bridge_boundary_components_with_polyline(panel.face, path)
            .map_err(map_split_error)?;

        let operation_id = OperationId(candidate.next_operation_id);
        for segment in path.windows(2) {
            candidate.seams.push(Seam {
                id: SeamId(candidate.next_seam_id),
                operation: operation_id,
                kind: OperationKind::Cut,
                start: segment[0],
                end: segment[1],
                panel_a: panel_id,
                panel_b: panel_id,
            });
            candidate.next_seam_id += 1;
        }
        candidate.operations.push(Operation {
            id: operation_id,
            kind: OperationKind::Cut,
            path: path.to_vec(),
        });
        candidate.next_operation_id += 1;
        *self = candidate;
        Ok(operation_id)
    }

    fn split_panel_with_path(
        &mut self,
        panel_id: PanelId,
        path: &[Point2],
        kind: OperationKind,
    ) -> Result<OperationId, ModelError> {
        validate_operation_path(path, kind)?;
        let mut candidate = self.clone();
        let operation_id = OperationId(candidate.next_operation_id);
        candidate.apply_panel_path_split(panel_id, path, kind, operation_id)?;
        candidate.operations.push(Operation {
            id: operation_id,
            kind,
            path: path.to_vec(),
        });
        candidate.next_operation_id += 1;
        *self = candidate;
        Ok(operation_id)
    }

    /// Applies one logical cut or straight crease across every face traversed by the
    /// path. Boundary intersections become deterministic seam endpoints while the
    /// operation identity remains stable across all generated panel splits.
    pub fn split_across_panels_with_polyline(
        &mut self,
        path: &[Point2],
        kind: OperationKind,
    ) -> Result<OperationId, ModelError> {
        validate_operation_path(path, kind)?;
        let fragments = self
            .topology
            .trace_polyline_across_faces(path)
            .map_err(map_split_error)?;
        let mut candidate = self.clone();
        let operation_id = OperationId(candidate.next_operation_id);

        for fragment in fragments {
            let face = candidate
                .topology
                .face_containing_fragment(&fragment)
                .map_err(map_split_error)?;
            let panel_id = candidate
                .panels
                .iter()
                .find(|panel| panel.face == face)
                .map(|panel| panel.id)
                .ok_or(ModelError::Topology(TopologyError::InvalidTopology))?;
            candidate.apply_panel_path_split(panel_id, &fragment, kind, operation_id)?;
        }

        candidate.operations.push(Operation {
            id: operation_id,
            kind,
            path: path.to_vec(),
        });
        candidate.next_operation_id += 1;
        *self = candidate;
        Ok(operation_id)
    }

    fn apply_panel_path_split(
        &mut self,
        panel_id: PanelId,
        path: &[Point2],
        kind: OperationKind,
        operation_id: OperationId,
    ) -> Result<(), ModelError> {
        let panel_index = self
            .panels
            .iter()
            .position(|panel| panel.id == panel_id)
            .ok_or(ModelError::UnknownPanel(panel_id))?;
        let original_face = self.panels[panel_index].face;
        let start = path[0];
        let end = *path.last().expect("path validated");
        let new_face = if path.len() == 2 {
            self.topology
                .split_face_with_segment(original_face, start, end)
                .map_err(map_split_error)?
        } else {
            self.topology
                .split_face_with_polyline(original_face, path)
                .map_err(map_split_error)?
        };
        let a_boundaries = self.topology.face_boundary_loops(original_face)?;
        let b_boundaries = self.topology.face_boundary_loops(new_face)?;

        let original_id = self.panels[panel_index].id;
        let new_id = PanelId(self.next_panel_id);
        let (reattached_seams, mut next_seam_id) = self.reattached_seams_after_split(
            original_id,
            new_id,
            start,
            end,
            &a_boundaries,
            &b_boundaries,
        )?;

        let mut new_seams = Vec::with_capacity(path.len() - 1);
        for segment in path.windows(2) {
            new_seams.push(Seam {
                id: SeamId(next_seam_id),
                operation: operation_id,
                kind,
                start: segment[0],
                end: segment[1],
                panel_a: original_id,
                panel_b: new_id,
            });
            next_seam_id += 1;
        }

        self.panels.push(Panel {
            id: new_id,
            face: new_face,
        });
        self.seams = reattached_seams;
        self.seams.extend(new_seams);
        self.next_panel_id += 1;
        self.next_seam_id = next_seam_id;
        Ok(())
    }

    pub fn render_snapshot(&self, fold: Option<FoldRequest>) -> Result<RenderSnapshot, ModelError> {
        match fold {
            Some(request) => self.render_snapshot_with_folds(&[request]),
            None => self.render_snapshot_with_folds(&[]),
        }
    }

    /// Renders a deterministic multi-crease fold state.
    ///
    /// Fold requests are canonicalized by operation identity so callers cannot change the
    /// result by reordering the same state. Each later hinge is resolved in world space after
    /// the earlier canonical folds. If those folds make a requested straight crease non-rigid
    /// (for example, a crossing crease becomes kinked), the state fails closed rather than
    /// inventing a physically inconsistent axis.
    pub fn render_snapshot_with_folds(
        &self,
        folds: &[FoldRequest],
    ) -> Result<RenderSnapshot, ModelError> {
        let fold_states = self.resolve_folds(folds)?;
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut triangle_panels = Vec::new();

        for panel in &self.panels {
            let triangulation = self.topology.triangulate_face(panel.face)?;
            let panel_start = vertices.len();
            for point in triangulation.vertices {
                let point_3d = Self::transform_point_for_panel(
                    &fold_states,
                    panel.id,
                    Vec3::new(point.x, point.y, 0.0),
                );
                vertices.push([point_3d.x, point_3d.y, point_3d.z]);
            }
            for triangle in triangulation.triangles {
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
        }

        let seams = self
            .seams
            .iter()
            .map(|seam| {
                let start = Self::transform_point_for_panel(
                    &fold_states,
                    seam.panel_a,
                    Vec3::new(seam.start.x, seam.start.y, 0.0),
                );
                let end = Self::transform_point_for_panel(
                    &fold_states,
                    seam.panel_a,
                    Vec3::new(seam.end.x, seam.end.y, 0.0),
                );
                RenderSeam {
                    operation: seam.operation,
                    kind: seam.kind,
                    start: [start.x, start.y, start.z],
                    end: [end.x, end.y, end.z],
                }
            })
            .collect();

        Ok(RenderSnapshot {
            vertices,
            indices,
            triangle_panels,
            seams,
            panel_count: self.panels.len(),
            component_count: self.component_count(),
        })
    }

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

    fn intentional_contact_pairs(&self) -> Result<HashSet<(PanelId, PanelId)>, ModelError> {
        let mut contacts: HashSet<(PanelId, PanelId)> = self
            .seams
            .iter()
            .filter(|seam| seam.panel_a != seam.panel_b)
            .map(|seam| canonical_panel_pair(seam.panel_a, seam.panel_b))
            .collect();
        for left_index in 0..self.panels.len() {
            for right_index in (left_index + 1)..self.panels.len() {
                let left = &self.panels[left_index];
                let right = &self.panels[right_index];
                if self.topology.faces_share_vertex(left.face, right.face)? {
                    contacts.insert(canonical_panel_pair(left.id, right.id));
                }
            }
        }
        Ok(contacts)
    }

    pub fn three_d_mesh(&self, fold: Option<FoldRequest>) -> Result<Mesh, MeshBuildError> {
        match fold {
            Some(request) => self.three_d_mesh_with_folds(&[request]),
            None => self.three_d_mesh_with_folds(&[]),
        }
    }

    pub fn three_d_mesh_with_folds(
        &self,
        folds: &[FoldRequest],
    ) -> Result<Mesh, MeshBuildError> {
        let snapshot = self
            .render_snapshot_with_folds(folds)
            .map_err(MeshBuildError::Model)?;
        let vertices = snapshot
            .vertices
            .into_iter()
            .map(|[x, y, z]| Vec3::new(x, y, z))
            .collect();
        Mesh::new(vertices, snapshot.indices).map_err(MeshBuildError::Mesh)
    }

    fn resolve_folds(&self, requests: &[FoldRequest]) -> Result<Vec<ResolvedFold>, ModelError> {
        let mut ordered = requests.to_vec();
        ordered.sort_by_key(|request| request.operation.0);
        if let Some(duplicate) = ordered
            .windows(2)
            .find(|pair| pair[0].operation == pair[1].operation)
        {
            return Err(ModelError::DuplicateFoldOperation(duplicate[0].operation));
        }

        let mut resolved = Vec::with_capacity(ordered.len());
        for request in ordered {
            if !request.angle_radians.is_finite() {
                return Err(ModelError::NonFiniteFoldAngle);
            }
            let operation = self
                .operations
                .iter()
                .find(|operation| operation.id == request.operation)
                .ok_or(ModelError::UnknownOperation(request.operation))?;
            if operation.kind == OperationKind::Cut {
                return Err(ModelError::CannotFoldCut(request.operation));
            }
            if !path_is_straight(&operation.path) {
                return Err(ModelError::BentCreaseRequiresConstraintSolver);
            }

            let moving_seeds: HashSet<PanelId> = self
                .seams
                .iter()
                .filter(|seam| seam.operation == operation.id)
                .map(|seam| seam.panel_b)
                .collect();
            let moving_panels =
                self.connected_panels_without_operation(&moving_seeds, operation.id);
            let (axis_start, axis_end) =
                self.transformed_operation_axis(operation.id, &resolved)?;

            resolved.push(ResolvedFold {
                operation: operation.id,
                axis_start,
                axis_end,
                angle_radians: request.angle_radians,
                moving_panels,
            });
        }
        Ok(resolved)
    }

    fn transformed_operation_axis(
        &self,
        operation: OperationId,
        resolved: &[ResolvedFold],
    ) -> Result<(Vec3, Vec3), ModelError> {
        let mut axis = None;
        let mut transformed_points = Vec::new();

        for seam in self.seams.iter().filter(|seam| seam.operation == operation) {
            let flat_start = Vec3::new(seam.start.x, seam.start.y, 0.0);
            let flat_end = Vec3::new(seam.end.x, seam.end.y, 0.0);
            let start_a = Self::transform_point_for_panel(resolved, seam.panel_a, flat_start);
            let end_a = Self::transform_point_for_panel(resolved, seam.panel_a, flat_end);
            let start_b = Self::transform_point_for_panel(resolved, seam.panel_b, flat_start);
            let end_b = Self::transform_point_for_panel(resolved, seam.panel_b, flat_end);

            if !points_near_3d(start_a, start_b) || !points_near_3d(end_a, end_b) {
                return Err(ModelError::FoldConstraintConflict(operation));
            }

            axis.get_or_insert((start_a, end_a));
            transformed_points.push(start_a);
            transformed_points.push(end_a);
        }

        let (axis_start, axis_end) = axis.ok_or(ModelError::FoldConstraintConflict(operation))?;
        if transformed_points
            .into_iter()
            .any(|point| !point_on_axis_3d(point, axis_start, axis_end))
        {
            return Err(ModelError::FoldConstraintConflict(operation));
        }
        Ok((axis_start, axis_end))
    }

    fn transform_point_for_panel(
        resolved: &[ResolvedFold],
        panel: PanelId,
        mut point: Vec3,
    ) -> Vec3 {
        for fold in resolved {
            if fold.moving_panels.contains(&panel) {
                point = fold.transform(point);
            }
        }
        point
    }

    fn connected_panels_without_operation(
        &self,
        seeds: &HashSet<PanelId>,
        excluded: OperationId,
    ) -> HashSet<PanelId> {
        let mut visited = seeds.clone();
        let mut queue: VecDeque<PanelId> = seeds.iter().copied().collect();
        while let Some(current) = queue.pop_front() {
            for seam in self
                .seams
                .iter()
                .filter(|seam| seam.kind == OperationKind::Crease && seam.operation != excluded)
            {
                let neighbor = adjacent_panel(seam, current);
                match neighbor {
                    Some(neighbor) if visited.insert(neighbor) => {
                        queue.push_back(neighbor);
                    }
                    _ => {}
                }
            }
        }
        visited
    }

    fn reattach_seams_inside_closed_cut(
        &mut self,
        original_panel: PanelId,
        new_panel: PanelId,
        closed_path: &[Point2],
    ) {
        let polygon = &closed_path[..closed_path.len() - 1];
        for seam in &mut self.seams {
            if seam.panel_a != original_panel && seam.panel_b != original_panel {
                continue;
            }
            let midpoint = interpolate(seam.start, seam.end, 0.5);
            if !point_in_polygon(midpoint, polygon) {
                continue;
            }
            if seam.panel_a == original_panel {
                seam.panel_a = new_panel;
            }
            if seam.panel_b == original_panel {
                seam.panel_b = new_panel;
            }
        }
    }

    fn reattached_seams_after_split(
        &self,
        original_panel: PanelId,
        new_panel: PanelId,
        split_start: Point2,
        split_end: Point2,
        a_boundaries: &FaceBoundaryLoops,
        b_boundaries: &FaceBoundaryLoops,
    ) -> Result<(Vec<Seam>, u32), ModelError> {
        let mut rebuilt = Vec::with_capacity(self.seams.len() + 2);
        let mut next_seam_id = self.next_seam_id;

        for seam in &self.seams {
            let replaces_a = seam.panel_a == original_panel;
            let replaces_b = seam.panel_b == original_panel;
            if !replaces_a && !replaces_b {
                rebuilt.push(seam.clone());
                continue;
            }

            let pieces = seam_pieces_after_split(
                seam,
                split_start,
                split_end,
                original_panel,
                new_panel,
                a_boundaries,
                b_boundaries,
            )?;
            for (piece_index, (start, end, child_panel)) in pieces.into_iter().enumerate() {
                let mut piece = seam.clone();
                if piece_index > 0 {
                    piece.id = SeamId(next_seam_id);
                    next_seam_id += 1;
                }
                piece.start = start;
                piece.end = end;
                if replaces_a {
                    piece.panel_a = child_panel;
                }
                if replaces_b {
                    piece.panel_b = child_panel;
                }
                rebuilt.push(piece);
            }
        }

        Ok((rebuilt, next_seam_id))
    }
}

#[derive(Debug)]
pub enum MeshBuildError {
    Model(ModelError),
    Mesh(MeshError),
}

impl fmt::Display for MeshBuildError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Model(error) => write!(formatter, "{error}"),
            Self::Mesh(error) => write!(
                formatter,
                "3d-lab mesh rejected kirigami geometry: {error:?}"
            ),
        }
    }
}

impl std::error::Error for MeshBuildError {}

struct ResolvedFold {
    operation: OperationId,
    axis_start: Vec3,
    axis_end: Vec3,
    angle_radians: f32,
    moving_panels: HashSet<PanelId>,
}

impl ResolvedFold {
    fn transform(&self, point: Vec3) -> Vec3 {
        rotate_around_axis(point, self.axis_start, self.axis_end, self.angle_radians)
    }
}

fn analyze_self_intersections(
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

fn collision_triangle_id(index: usize) -> Result<u32, ModelError> {
    u32::try_from(index).map_err(|_| ModelError::TooManyCollisionTriangles)
}

fn triangle_vertices(snapshot: &RenderSnapshot, triangle_index: usize) -> [[f32; 3]; 3] {
    let offset = triangle_index * 3;
    std::array::from_fn(|corner| snapshot.vertices[snapshot.indices[offset + corner] as usize])
}

fn canonical_panel_pair(left: PanelId, right: PanelId) -> (PanelId, PanelId) {
    if left.0 <= right.0 {
        (left, right)
    } else {
        (right, left)
    }
}

fn validate_operation_path(path: &[Point2], kind: OperationKind) -> Result<(), ModelError> {
    if path.len() < 2 || path.iter().any(|point| !point.is_finite()) {
        return Err(ModelError::InvalidPath);
    }
    if path
        .windows(2)
        .any(|segment| squared_distance(segment[0], segment[1]) <= EPSILON * EPSILON)
    {
        return Err(ModelError::DegenerateSegment);
    }
    if kind == OperationKind::Crease && !path_is_straight(path) {
        return Err(ModelError::BentCreaseRequiresConstraintSolver);
    }
    Ok(())
}

fn map_split_error(error: TopologyError) -> ModelError {
    match error {
        TopologyError::PointOffBoundary => ModelError::SegmentEndpointOffBoundary,
        TopologyError::DegenerateSegment => ModelError::DegenerateSegment,
        TopologyError::TooFewPathPoints => ModelError::InvalidPath,
        TopologyError::PolylineSelfIntersecting
        | TopologyError::PolylineLeavesFace
        | TopologyError::PolylineLeavesTopology
        | TopologyError::PolylineOverlapsBoundary
        | TopologyError::PolylineCrossingAmbiguous
        | TopologyError::BoundaryBridgeRequiresDistinctComponents
        | TopologyError::SegmentDoesNotSplitFace
        | TopologyError::SegmentLeavesFace => ModelError::SegmentDoesNotSplitPanel,
        other => ModelError::Topology(other),
    }
}

fn path_is_straight(path: &[Point2]) -> bool {
    if path.len() < 2 {
        return false;
    }
    let start = path[0];
    let end = *path.last().expect("path length checked");
    let axis_x = end.x - start.x;
    let axis_y = end.y - start.y;
    let axis_length_squared = axis_x * axis_x + axis_y * axis_y;
    if axis_length_squared <= EPSILON * EPSILON {
        return false;
    }
    let axis_length = axis_length_squared.sqrt();
    path[1..path.len() - 1].iter().all(|point| {
        let cross = axis_x * (point.y - start.y) - axis_y * (point.x - start.x);
        cross.abs() <= EPSILON * axis_length
    })
}

fn adjacent_panel(seam: &Seam, panel: PanelId) -> Option<PanelId> {
    if seam.panel_a == panel {
        Some(seam.panel_b)
    } else if seam.panel_b == panel {
        Some(seam.panel_a)
    } else {
        None
    }
}

fn seam_pieces_after_split(
    seam: &Seam,
    split_start: Point2,
    split_end: Point2,
    original_panel: PanelId,
    new_panel: PanelId,
    a_boundaries: &FaceBoundaryLoops,
    b_boundaries: &FaceBoundaryLoops,
) -> Result<Vec<(Point2, Point2, PanelId)>, ModelError> {
    let mut parameters = vec![0.0, 1.0];
    for split_point in [split_start, split_end] {
        if point_on_segment(split_point, seam.start, seam.end) {
            let parameter = segment_parameter(split_point, seam.start, seam.end);
            if parameter > EPSILON && parameter < 1.0 - EPSILON {
                parameters.push(parameter);
            }
        }
    }
    parameters.sort_by(|left, right| {
        left.partial_cmp(right)
            .expect("finite seam split parameters")
    });
    parameters.dedup_by(|left, right| (*left - *right).abs() <= EPSILON);

    let mut pieces = Vec::with_capacity(parameters.len().saturating_sub(1));
    for window in parameters.windows(2) {
        let start = interpolate(seam.start, seam.end, window[0]);
        let end = interpolate(seam.start, seam.end, window[1]);
        if squared_distance(start, end) <= EPSILON * EPSILON {
            continue;
        }
        let midpoint = interpolate(start, end, 0.5);
        let on_a = point_on_boundary_loops(midpoint, a_boundaries);
        let on_b = point_on_boundary_loops(midpoint, b_boundaries);
        let panel = match (on_a, on_b) {
            (true, false) => original_panel,
            (false, true) => new_panel,
            _ => return Err(ModelError::SeamReattachmentFailed(seam.id)),
        };
        pieces.push((start, end, panel));
    }

    if pieces.is_empty() {
        return Err(ModelError::SeamReattachmentFailed(seam.id));
    }
    Ok(pieces)
}

fn point_on_boundary_loops(point: Point2, boundaries: &FaceBoundaryLoops) -> bool {
    point_on_boundary(point, &boundaries.outer)
        || boundaries
            .holes
            .iter()
            .any(|hole| point_on_boundary(point, hole))
}

fn point_on_boundary(point: Point2, polygon: &[Point2]) -> bool {
    (0..polygon.len())
        .any(|index| point_on_segment(point, polygon[index], polygon[(index + 1) % polygon.len()]))
}

fn point_in_polygon(point: Point2, polygon: &[Point2]) -> bool {
    let mut inside = false;
    let mut previous = polygon.len() - 1;
    for current in 0..polygon.len() {
        let a = polygon[current];
        let b = polygon[previous];
        let crosses = (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
        if crosses {
            inside = !inside;
        }
        previous = current;
    }
    inside
}

fn canonical_closed_path(path: &[Point2]) -> Result<Vec<Point2>, ModelError> {
    if path.len() < 3 || path.iter().any(|point| !point.is_finite()) {
        return Err(ModelError::InvalidPath);
    }
    let mut canonical = path.to_vec();
    if squared_distance(canonical[0], *canonical.last().expect("path is non-empty"))
        <= EPSILON * EPSILON
    {
        canonical.pop();
    }
    if canonical.len() < 3 {
        return Err(ModelError::InvalidPath);
    }
    for index in 0..canonical.len() {
        if squared_distance(canonical[index], canonical[(index + 1) % canonical.len()])
            <= EPSILON * EPSILON
        {
            return Err(ModelError::DegenerateSegment);
        }
    }
    canonical.push(canonical[0]);
    Ok(canonical)
}

fn point_on_segment(point: Point2, start: Point2, end: Point2) -> bool {
    let ab = Point2::new(end.x - start.x, end.y - start.y);
    let ap = Point2::new(point.x - start.x, point.y - start.y);
    let cross = ab.x * ap.y - ab.y * ap.x;
    if cross.abs() > EPSILON {
        return false;
    }
    let dot = ap.x * ab.x + ap.y * ab.y;
    let length_squared = ab.x * ab.x + ab.y * ab.y;
    dot >= -EPSILON && dot <= length_squared + EPSILON
}

fn segment_parameter(point: Point2, start: Point2, end: Point2) -> f32 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_squared = dx * dx + dy * dy;
    ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
}

fn interpolate(start: Point2, end: Point2, t: f32) -> Point2 {
    Point2::new(
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t,
    )
}

fn squared_distance(a: Point2, b: Point2) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

fn rotate_around_axis(point: Vec3, axis_start: Vec3, axis_end: Vec3, angle: f32) -> Vec3 {
    let Some(axis) = (axis_end - axis_start).normalized() else {
        return point;
    };
    let relative = point - axis_start;
    let cosine = angle.cos();
    let sine = angle.sin();
    axis_start
        + relative * cosine
        + axis.cross(relative) * sine
        + axis * (axis.dot(relative) * (1.0 - cosine))
}

fn points_near_3d(left: Vec3, right: Vec3) -> bool {
    let delta = left - right;
    delta.dot(delta) <= EPSILON * EPSILON * 16.0
}

fn point_on_axis_3d(point: Vec3, axis_start: Vec3, axis_end: Vec3) -> bool {
    let Some(axis) = (axis_end - axis_start).normalized() else {
        return false;
    };
    let offset = point - axis_start;
    let perpendicular = offset - axis * axis.dot(offset);
    perpendicular.dot(perpendicular) <= EPSILON * EPSILON * 16.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn split(kind: OperationKind) -> (PaperModel, OperationId) {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let operation = model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -0.5),
                Point2::new(0.0, 0.5),
                kind,
            )
            .unwrap();
        (model, operation)
    }

    #[test]
    fn crease_splits_topology_but_preserves_connectivity() {
        let (model, _) = split(OperationKind::Crease);
        assert_eq!(model.panels().len(), 2);
        assert_eq!(model.topology().face_count(), 2);
        assert_eq!(model.component_count(), 1);
        model.topology().validate().unwrap();
    }

    #[test]
    fn cut_splits_connectivity() {
        let (model, operation) = split(OperationKind::Cut);
        assert_eq!(model.panels().len(), 2);
        assert_eq!(model.component_count(), 2);
        assert!(matches!(
            model.render_snapshot(Some(FoldRequest {
                operation,
                angle_radians: 0.5
            })),
            Err(ModelError::CannotFoldCut(_))
        ));
    }

    #[test]
    fn crease_fold_produces_depth_and_three_d_lab_mesh() {
        let (model, operation) = split(OperationKind::Crease);
        let fold = FoldRequest {
            operation,
            angle_radians: std::f32::consts::FRAC_PI_2,
        };
        let snapshot = model.render_snapshot(Some(fold)).unwrap();
        assert!(snapshot.vertices.iter().any(|vertex| vertex[2].abs() > 0.5));
        assert_eq!(snapshot.indices.len(), 12);
        assert_eq!(model.three_d_mesh(Some(fold)).unwrap().triangle_count(), 4);
    }

    #[test]
    fn multiple_parallel_creases_propagate_deterministically() {
        let mut model = PaperModel::rectangle(3.0, 1.0).unwrap();
        let first = model
            .split_across_panels_with_polyline(
                &[Point2::new(-0.5, -0.5), Point2::new(-0.5, 0.5)],
                OperationKind::Crease,
            )
            .unwrap();
        let second = model
            .split_across_panels_with_polyline(
                &[Point2::new(0.5, -0.5), Point2::new(0.5, 0.5)],
                OperationKind::Crease,
            )
            .unwrap();

        let folds = [
            FoldRequest {
                operation: first,
                angle_radians: 0.7,
            },
            FoldRequest {
                operation: second,
                angle_radians: -0.45,
            },
        ];
        let forward = model.render_snapshot_with_folds(&folds).unwrap();
        let reverse = model
            .render_snapshot_with_folds(&[folds[1], folds[0]])
            .unwrap();

        assert_eq!(forward, reverse);
        assert!(forward.vertices.iter().any(|vertex| vertex[2].abs() > 0.25));
        assert_eq!(
            model.three_d_mesh_with_folds(&folds).unwrap().triangle_count(),
            6
        );
    }

    #[test]
    fn crossing_crease_state_fails_closed_when_axis_becomes_kinked() {
        let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
        let vertical = model
            .split_across_panels_with_polyline(
                &[Point2::new(0.0, -1.0), Point2::new(0.0, 1.0)],
                OperationKind::Crease,
            )
            .unwrap();
        let horizontal = model
            .split_across_panels_with_polyline(
                &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
                OperationKind::Crease,
            )
            .unwrap();

        assert!(matches!(
            model.render_snapshot_with_folds(&[
                FoldRequest {
                    operation: vertical,
                    angle_radians: 0.6,
                },
                FoldRequest {
                    operation: horizontal,
                    angle_radians: 0.4,
                },
            ]),
            Err(ModelError::FoldConstraintConflict(operation)) if operation == horizontal
        ));
    }

    #[test]
    fn duplicate_fold_requests_fail_closed() {
        let (model, operation) = split(OperationKind::Crease);
        assert!(matches!(
            model.render_snapshot_with_folds(&[
                FoldRequest {
                    operation,
                    angle_radians: 0.4,
                },
                FoldRequest {
                    operation,
                    angle_radians: 0.5,
                },
            ]),
            Err(ModelError::DuplicateFoldOperation(duplicate)) if duplicate == operation
        ));
    }

    #[test]
    fn split_rejects_non_boundary_endpoints_without_mutation() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let before = model.clone();
        assert_eq!(
            model.split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, 0.0),
                Point2::new(0.0, 0.5),
                OperationKind::Cut,
            ),
            Err(ModelError::SegmentEndpointOffBoundary)
        );
        assert_eq!(model, before);
    }

    #[test]
    fn later_split_preserves_existing_crease_adjacency() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let crease = model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -0.5),
                Point2::new(0.0, 0.5),
                OperationKind::Crease,
            )
            .unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(-1.0, 0.0),
                Point2::new(0.0, 0.0),
                OperationKind::Cut,
            )
            .unwrap();

        let crease_segments: Vec<&Seam> = model
            .seams()
            .iter()
            .filter(|seam| seam.operation == crease)
            .collect();
        assert_eq!(crease_segments.len(), 2);
        assert_eq!(model.component_count(), 1);
        assert_eq!(model.topology().face_count(), 3);
        model.topology().validate().unwrap();
        assert!(
            crease_segments
                .iter()
                .any(|seam| seam.panel_a == PanelId(0))
        );
        assert!(
            crease_segments
                .iter()
                .any(|seam| seam.panel_a == PanelId(2))
        );
    }

    #[test]
    fn folding_subdivided_crease_excludes_all_of_its_segments() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let crease = model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -0.5),
                Point2::new(0.0, 0.5),
                OperationKind::Crease,
            )
            .unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(-1.0, 0.0),
                Point2::new(0.0, 0.0),
                OperationKind::Cut,
            )
            .unwrap();

        let snapshot = model
            .render_snapshot(Some(FoldRequest {
                operation: crease,
                angle_radians: std::f32::consts::FRAC_PI_2,
            }))
            .unwrap();
        assert!(snapshot.vertices.iter().any(|vertex| vertex[2].abs() > 0.5));
    }

    #[test]
    fn cut_across_existing_crease_is_one_operation() {
        let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
        let first_crease = model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
                OperationKind::Crease,
            )
            .unwrap();
        let cut = model
            .split_across_panels_with_polyline(
                &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
                OperationKind::Cut,
            )
            .unwrap();

        assert_eq!(model.panels().len(), 4);
        assert_eq!(model.topology().face_count(), 4);
        assert_eq!(model.component_count(), 2);
        assert_eq!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == cut)
                .count(),
            2
        );
        assert_eq!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == first_crease)
                .count(),
            2
        );
        assert_eq!(model.operations().last().unwrap().path.len(), 2);
        model.topology().validate().unwrap();
    }

    #[test]
    fn bent_cut_across_existing_crease_preserves_one_operation() {
        let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
                OperationKind::Crease,
            )
            .unwrap();
        let path = [
            Point2::new(-1.0, -0.5),
            Point2::new(-0.25, 0.25),
            Point2::new(0.75, 0.25),
            Point2::new(1.0, -0.5),
        ];
        let cut = model
            .split_across_panels_with_polyline(&path, OperationKind::Cut)
            .unwrap();

        assert_eq!(model.panels().len(), 4);
        assert_eq!(model.component_count(), 2);
        assert_eq!(model.operations().last().unwrap().path, path);
        assert_eq!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == cut)
                .count(),
            4
        );
        model.topology().validate().unwrap();
    }

    #[test]
    fn straight_crease_can_cross_existing_crease_with_one_fold_side() {
        let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
                OperationKind::Crease,
            )
            .unwrap();
        let horizontal = model
            .split_across_panels_with_polyline(
                &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
                OperationKind::Crease,
            )
            .unwrap();

        assert_eq!(model.component_count(), 1);
        assert_eq!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == horizontal)
                .count(),
            2
        );
        let resolved = model
            .resolve_fold(FoldRequest {
                operation: horizontal,
                angle_radians: std::f32::consts::FRAC_PI_2,
            })
            .unwrap();
        assert_eq!(resolved.moving_panels.len(), 2);
        let snapshot = model
            .render_snapshot(Some(FoldRequest {
                operation: horizontal,
                angle_radians: std::f32::consts::FRAC_PI_2,
            }))
            .unwrap();
        assert!(snapshot.vertices.iter().any(|vertex| vertex[2].abs() > 0.5));
    }

    #[test]
    fn multi_face_path_failure_is_transactional() {
        let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
                OperationKind::Crease,
            )
            .unwrap();
        let before = model.clone();
        assert_eq!(
            model.split_across_panels_with_polyline(
                &[
                    Point2::new(-1.0, 0.0),
                    Point2::new(0.0, 0.0),
                    Point2::new(0.0, 0.75),
                ],
                OperationKind::Cut,
            ),
            Err(ModelError::SegmentDoesNotSplitPanel)
        );
        assert_eq!(model, before);
    }

    #[test]
    fn closed_cut_creates_detached_panel_and_hole() {
        let mut model = PaperModel::rectangle(4.0, 4.0).unwrap();
        let cut = model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(-0.5, -0.5),
                    Point2::new(0.5, -0.5),
                    Point2::new(0.5, 0.5),
                    Point2::new(-0.5, 0.5),
                ],
            )
            .unwrap();

        assert_eq!(model.panels().len(), 2);
        assert_eq!(model.component_count(), 2);
        assert_eq!(
            model.operations().last().unwrap().path.first(),
            model.operations().last().unwrap().path.last()
        );
        assert_eq!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == cut)
                .count(),
            4
        );
        assert_eq!(
            model
                .topology()
                .face_boundary_loops(FaceId(0))
                .unwrap()
                .holes
                .len(),
            1
        );
        let snapshot = model.render_snapshot(None).unwrap();
        assert_eq!(snapshot.panel_count, 2);
        assert_eq!(snapshot.component_count, 2);
    }

    #[test]
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
        let report = analyze_self_intersections(&snapshot, &HashSet::new()).unwrap();
        assert_eq!(report.triangle_count, 2);
        assert_eq!(report.narrow_phase_tests, 1);
        assert_eq!(report.intersections.len(), 1);
        assert!(report.indeterminate_pairs.is_empty());
        assert!(!report.is_proven_clear());
    }

    #[test]
    fn contact_filter_allows_intended_neighbor_boundary_contact() {
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
        let report = analyze_self_intersections(
            &snapshot,
            &HashSet::from([canonical_panel_pair(neighbor.panel_a, neighbor.panel_b)]),
        )
        .unwrap();
        assert_eq!(
            report.scope,
            SelfIntersectionScope::AllPanelPairsContactAware
        );
        assert!(report.is_proven_clear());
        assert!(report.narrow_phase_tests >= 1);
    }

    #[test]
    fn adjacent_fold_contact_is_clear_but_full_overlap_is_reported() {
        let mut model = PaperModel::rectangle(4.0, 2.0).unwrap();
        let operation = model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
                OperationKind::Crease,
            )
            .unwrap();

        let hinge_contact = model
            .self_intersection_report(Some(FoldRequest {
                operation,
                angle_radians: std::f32::consts::FRAC_PI_2,
            }))
            .unwrap();
        assert!(hinge_contact.is_proven_clear());

        let overlap = model
            .self_intersection_report(Some(FoldRequest {
                operation,
                angle_radians: std::f32::consts::PI,
            }))
            .unwrap();
        assert!(!overlap.is_proven_clear());
        assert!(!overlap.intersections.is_empty());
    }

    #[test]
    fn crossing_creases_shared_vertex_is_intentional_contact() {
        let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
                OperationKind::Crease,
            )
            .unwrap();
        model
            .split_across_panels_with_polyline(
                &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
                OperationKind::Crease,
            )
            .unwrap();

        let contacts = model.intentional_contact_pairs().unwrap();
        assert!(contacts.contains(&canonical_panel_pair(PanelId(0), PanelId(3))));
        let report = model.self_intersection_report(None).unwrap();
        assert!(report.is_proven_clear());
        assert!(report.intersections.is_empty());
        assert!(report.indeterminate_pairs.is_empty());
    }

    #[test]
    fn collision_triangle_ids_fail_closed_before_truncation() {
        assert_eq!(
            collision_triangle_id(u32::MAX as usize + 1),
            Err(ModelError::TooManyCollisionTriangles)
        );
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
    fn boundary_bridge_cut_opens_annulus_without_new_panel() {
        let mut model = PaperModel::rectangle(4.0, 4.0).unwrap();
        model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(-0.5, -0.5),
                    Point2::new(0.5, -0.5),
                    Point2::new(0.5, 0.5),
                    Point2::new(-0.5, 0.5),
                ],
            )
            .unwrap();
        let panel_count = model.panels().len();
        let bridge = model
            .cut_boundary_bridge(
                PanelId(0),
                &[Point2::new(-2.0, 0.0), Point2::new(-0.5, 0.0)],
            )
            .unwrap();

        assert_eq!(model.panels().len(), panel_count);
        assert_eq!(model.component_count(), 2);
        assert_eq!(
            model
                .topology()
                .boundary_component_count(FaceId(0))
                .unwrap(),
            1
        );
        let bridge_segments: Vec<&Seam> = model
            .seams()
            .iter()
            .filter(|seam| seam.operation == bridge)
            .collect();
        assert_eq!(bridge_segments.len(), 1);
        assert!(
            bridge_segments
                .iter()
                .all(|seam| seam.panel_a == PanelId(0) && seam.panel_b == PanelId(0))
        );
        assert!(model.render_snapshot(None).is_ok());
    }

    #[test]
    fn boundary_bridge_can_merge_two_holes_without_new_panel() {
        let mut model = PaperModel::rectangle(6.0, 4.0).unwrap();
        model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(-1.75, -0.5),
                    Point2::new(-0.75, -0.5),
                    Point2::new(-0.75, 0.5),
                    Point2::new(-1.75, 0.5),
                ],
            )
            .unwrap();
        model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(0.75, -0.5),
                    Point2::new(1.75, -0.5),
                    Point2::new(1.75, 0.5),
                    Point2::new(0.75, 0.5),
                ],
            )
            .unwrap();
        let before = model.panels().len();
        model
            .cut_boundary_bridge(
                PanelId(0),
                &[Point2::new(-0.75, 0.0), Point2::new(0.75, 0.0)],
            )
            .unwrap();

        assert_eq!(model.panels().len(), before);
        assert_eq!(
            model
                .topology()
                .boundary_component_count(FaceId(0))
                .unwrap(),
            2
        );
        assert_eq!(model.component_count(), 3);
        assert!(model.render_snapshot(None).is_ok());
    }

    #[test]
    fn second_disjoint_closed_cut_adds_second_hole() {
        let mut model = PaperModel::rectangle(6.0, 4.0).unwrap();
        model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(-2.0, -0.5),
                    Point2::new(-1.0, -0.5),
                    Point2::new(-1.0, 0.5),
                    Point2::new(-2.0, 0.5),
                ],
            )
            .unwrap();
        model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(1.0, -0.5),
                    Point2::new(2.0, -0.5),
                    Point2::new(2.0, 0.5),
                    Point2::new(1.0, 0.5),
                ],
            )
            .unwrap();

        assert_eq!(model.panels().len(), 3);
        assert_eq!(model.component_count(), 3);
        assert_eq!(
            model
                .topology()
                .face_boundary_loops(FaceId(0))
                .unwrap()
                .holes
                .len(),
            2
        );
        assert!(model.render_snapshot(None).is_ok());
    }

    #[test]
    fn enclosing_closed_cut_moves_existing_cut_boundary_to_new_panel() {
        let mut model = PaperModel::rectangle(6.0, 6.0).unwrap();
        let inner_cut = model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(-0.5, -0.5),
                    Point2::new(0.5, -0.5),
                    Point2::new(0.5, 0.5),
                    Point2::new(-0.5, 0.5),
                ],
            )
            .unwrap();
        model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(-1.5, -1.5),
                    Point2::new(1.5, -1.5),
                    Point2::new(1.5, 1.5),
                    Point2::new(-1.5, 1.5),
                ],
            )
            .unwrap();

        let annulus_panel = PanelId(2);
        assert!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == inner_cut)
                .all(|seam| seam.panel_a == annulus_panel || seam.panel_b == annulus_panel)
        );
        assert_eq!(
            model
                .topology()
                .face_boundary_loops(
                    model
                        .panels()
                        .iter()
                        .find(|panel| panel.id == annulus_panel)
                        .unwrap()
                        .face
                )
                .unwrap()
                .holes
                .len(),
            1
        );
        model.topology().validate().unwrap();
    }

    #[test]
    fn open_cut_after_closed_cut_preserves_hole_seam_provenance() {
        let mut model = PaperModel::rectangle(6.0, 4.0).unwrap();
        let closed = model
            .cut_closed_path(
                PanelId(0),
                &[
                    Point2::new(-2.0, -0.5),
                    Point2::new(-1.0, -0.5),
                    Point2::new(-1.0, 0.5),
                    Point2::new(-2.0, 0.5),
                ],
            )
            .unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.5, -2.0),
                Point2::new(0.5, 2.0),
                OperationKind::Cut,
            )
            .unwrap();

        assert_eq!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == closed)
                .count(),
            4
        );
        assert!(model.render_snapshot(None).is_ok());
        model.topology().validate().unwrap();
    }

    #[test]
    fn bent_cut_creates_segmented_operation_and_disconnects() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let operation = model
            .split_panel_with_polyline(
                PanelId(0),
                &[
                    Point2::new(-1.0, 0.0),
                    Point2::new(0.0, 0.25),
                    Point2::new(1.0, 0.0),
                ],
                OperationKind::Cut,
            )
            .unwrap();

        assert_eq!(model.panels().len(), 2);
        assert_eq!(model.component_count(), 2);
        assert_eq!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == operation)
                .count(),
            2
        );
        assert_eq!(model.operations()[0].path.len(), 3);
        model.topology().validate().unwrap();
    }

    #[test]
    fn bent_crease_is_deferred_without_mutation() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let before = model.clone();
        assert_eq!(
            model.split_panel_with_polyline(
                PanelId(0),
                &[
                    Point2::new(-1.0, 0.0),
                    Point2::new(0.0, 0.25),
                    Point2::new(1.0, 0.0),
                ],
                OperationKind::Crease,
            ),
            Err(ModelError::BentCreaseRequiresConstraintSolver)
        );
        assert_eq!(model, before);
    }

    #[test]
    fn collinear_segmented_crease_still_uses_rigid_fold_axis() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let crease = model
            .split_panel_with_polyline(
                PanelId(0),
                &[
                    Point2::new(0.0, -0.5),
                    Point2::new(0.0, 0.0),
                    Point2::new(0.0, 0.5),
                ],
                OperationKind::Crease,
            )
            .unwrap();
        let snapshot = model
            .render_snapshot(Some(FoldRequest {
                operation: crease,
                angle_radians: std::f32::consts::FRAC_PI_2,
            }))
            .unwrap();
        assert!(snapshot.vertices.iter().any(|vertex| vertex[2].abs() > 0.5));
        assert_eq!(
            model
                .seams()
                .iter()
                .filter(|seam| seam.operation == crease)
                .count(),
            2
        );
    }

    #[test]
    fn downstream_crease_moves_with_folded_component() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let first = model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -0.5),
                Point2::new(0.0, 0.5),
                OperationKind::Crease,
            )
            .unwrap();
        let second = model
            .split_panel_with_segment(
                PanelId(1),
                Point2::new(0.5, -0.5),
                Point2::new(0.5, 0.5),
                OperationKind::Crease,
            )
            .unwrap();

        let snapshot = model
            .render_snapshot(Some(FoldRequest {
                operation: first,
                angle_radians: std::f32::consts::FRAC_PI_2,
            }))
            .unwrap();
        let downstream = snapshot
            .seams
            .iter()
            .find(|seam| seam.operation == second)
            .unwrap();
        assert!(downstream.start[2].abs() > 0.4);
        assert!(downstream.end[2].abs() > 0.4);
    }
}
