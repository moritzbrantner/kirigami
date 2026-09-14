//! Authoritative kirigami sheet topology and fold semantics.
//!
//! Rendering, browser interaction, and physical material simulation are consumers of
//! this crate. Cuts and creases are topology operations here, not renderer effects.

mod topology;

pub use topology::{
    FaceId, FaceTriangulation, HalfEdgeId, PlanarTopology, TopologyError, VertexId,
};

use serde::Serialize;
use std::collections::{HashSet, VecDeque};
use std::fmt;
use three_d_core::{Mesh, MeshError, Vec3};

const EPSILON: f32 = 1.0e-5;

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
    pub start: Point2,
    pub end: Point2,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Seam {
    pub id: SeamId,
    pub operation: OperationId,
    pub kind: OperationKind,
    pub start: Point2,
    pub end: Point2,
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
    pub seams: Vec<RenderSeam>,
    pub panel_count: usize,
    pub component_count: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ModelError {
    InvalidSheetDimensions,
    NonFinitePoint,
    DegenerateSegment,
    UnknownPanel(PanelId),
    UnknownOperation(OperationId),
    SegmentEndpointOffBoundary,
    SegmentDoesNotSplitPanel,
    CannotFoldCut(OperationId),
    NonFiniteFoldAngle,
    SeamReattachmentFailed(SeamId),
    Topology(TopologyError),
    TooManyRenderVertices,
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
    ///
    /// The geometric subdivision is owned by `PlanarTopology`. A logical operation can
    /// own multiple seam segments after later panel splits, so topology can evolve
    /// without changing the identity of an existing cut or crease.
    pub fn split_panel_with_segment(
        &mut self,
        panel_id: PanelId,
        start: Point2,
        end: Point2,
        kind: OperationKind,
    ) -> Result<OperationId, ModelError> {
        if !start.is_finite() || !end.is_finite() {
            return Err(ModelError::NonFinitePoint);
        }
        if squared_distance(start, end) <= EPSILON * EPSILON {
            return Err(ModelError::DegenerateSegment);
        }
        let panel_index = self
            .panels
            .iter()
            .position(|panel| panel.id == panel_id)
            .ok_or(ModelError::UnknownPanel(panel_id))?;
        let original_face = self.panels[panel_index].face;

        // Keep edits fail-closed: geometric subdivision and seam reassignment are
        // prepared against a candidate topology and committed only if both succeed.
        let mut candidate_topology = self.topology.clone();
        let new_face = candidate_topology
            .split_face_with_segment(original_face, start, end)
            .map_err(map_split_error)?;
        let a_polygon = candidate_topology.face_polygon(original_face)?;
        let b_polygon = candidate_topology.face_polygon(new_face)?;

        let original_id = self.panels[panel_index].id;
        let new_id = PanelId(self.next_panel_id);
        let (reattached_seams, next_seam_id) = self.reattached_seams_after_split(
            original_id,
            new_id,
            start,
            end,
            &a_polygon,
            &b_polygon,
        )?;

        let operation_id = OperationId(self.next_operation_id);
        let seam_id = SeamId(next_seam_id);
        self.topology = candidate_topology;
        self.panels.push(Panel {
            id: new_id,
            face: new_face,
        });
        self.seams = reattached_seams;
        self.seams.push(Seam {
            id: seam_id,
            operation: operation_id,
            kind,
            start,
            end,
            panel_a: original_id,
            panel_b: new_id,
        });
        self.operations.push(Operation {
            id: operation_id,
            kind,
            start,
            end,
        });
        self.next_panel_id += 1;
        self.next_operation_id += 1;
        self.next_seam_id = next_seam_id + 1;
        Ok(operation_id)
    }

    pub fn render_snapshot(&self, fold: Option<FoldRequest>) -> Result<RenderSnapshot, ModelError> {
        let fold_state = match fold {
            Some(request) => Some(self.resolve_fold(request)?),
            None => None,
        };
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        for panel in &self.panels {
            let triangulation = self.topology.triangulate_face(panel.face)?;
            let panel_start = vertices.len();
            let should_rotate = fold_state
                .as_ref()
                .is_some_and(|state| state.moving_panels.contains(&panel.id));
            for point in triangulation.vertices {
                let mut point_3d = Vec3::new(point.x, point.y, 0.0);
                if should_rotate {
                    let state = fold_state.as_ref().expect("fold state checked above");
                    point_3d = state.transform(point_3d);
                }
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
            }
        }

        let seams = self
            .seams
            .iter()
            .map(|seam| {
                let mut start = Vec3::new(seam.start.x, seam.start.y, 0.0);
                let mut end = Vec3::new(seam.end.x, seam.end.y, 0.0);
                if let Some(state) = &fold_state {
                    let seam_moves = seam.operation != state.operation
                        && state.moving_panels.contains(&seam.panel_a)
                        && state.moving_panels.contains(&seam.panel_b);
                    if seam_moves {
                        start = state.transform(start);
                        end = state.transform(end);
                    }
                }
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
            seams,
            panel_count: self.panels.len(),
            component_count: self.component_count(),
        })
    }

    pub fn three_d_mesh(&self, fold: Option<FoldRequest>) -> Result<Mesh, MeshBuildError> {
        let snapshot = self.render_snapshot(fold).map_err(MeshBuildError::Model)?;
        let vertices = snapshot
            .vertices
            .into_iter()
            .map(|[x, y, z]| Vec3::new(x, y, z))
            .collect();
        Mesh::new(vertices, snapshot.indices).map_err(MeshBuildError::Mesh)
    }

    fn resolve_fold(&self, request: FoldRequest) -> Result<ResolvedFold, ModelError> {
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

        let moving_seeds: HashSet<PanelId> = self
            .seams
            .iter()
            .filter(|seam| seam.operation == operation.id)
            .map(|seam| seam.panel_b)
            .collect();
        let moving_panels = self.connected_panels_without_operation(&moving_seeds, operation.id);
        Ok(ResolvedFold {
            operation: operation.id,
            axis_start: Vec3::new(operation.start.x, operation.start.y, 0.0),
            axis_end: Vec3::new(operation.end.x, operation.end.y, 0.0),
            angle_radians: request.angle_radians,
            moving_panels,
        })
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

    fn reattached_seams_after_split(
        &self,
        original_panel: PanelId,
        new_panel: PanelId,
        split_start: Point2,
        split_end: Point2,
        a_polygon: &[Point2],
        b_polygon: &[Point2],
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
                a_polygon,
                b_polygon,
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

fn map_split_error(error: TopologyError) -> ModelError {
    match error {
        TopologyError::PointOffBoundary => ModelError::SegmentEndpointOffBoundary,
        TopologyError::DegenerateSegment => ModelError::DegenerateSegment,
        TopologyError::SegmentDoesNotSplitFace | TopologyError::SegmentLeavesFace => {
            ModelError::SegmentDoesNotSplitPanel
        }
        other => ModelError::Topology(other),
    }
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
    a_polygon: &[Point2],
    b_polygon: &[Point2],
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
        let on_a = point_on_boundary(midpoint, a_polygon);
        let on_b = point_on_boundary(midpoint, b_polygon);
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

fn point_on_boundary(point: Point2, polygon: &[Point2]) -> bool {
    (0..polygon.len())
        .any(|index| point_on_segment(point, polygon[index], polygon[(index + 1) % polygon.len()]))
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
