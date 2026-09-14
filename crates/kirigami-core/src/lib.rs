//! Authoritative kirigami sheet topology and fold semantics.
//!
//! Rendering, browser interaction, and physical material simulation are consumers of
//! this crate. Cuts and creases are topology operations here, not renderer effects.

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
pub struct SeamId(pub u32);

#[derive(Debug, Clone, PartialEq)]
pub struct Panel {
    pub id: PanelId,
    pub vertices: Vec<usize>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Seam {
    pub id: SeamId,
    pub kind: OperationKind,
    pub start: Point2,
    pub end: Point2,
    pub panel_a: PanelId,
    pub panel_b: PanelId,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PaperModel {
    vertices: Vec<Point2>,
    panels: Vec<Panel>,
    seams: Vec<Seam>,
    next_panel_id: u32,
    next_seam_id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FoldRequest {
    pub seam: SeamId,
    pub angle_radians: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RenderSeam {
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
    UnknownSeam(SeamId),
    SegmentEndpointOffBoundary,
    SegmentDoesNotSplitPanel,
    CannotFoldCut(SeamId),
    NonFiniteFoldAngle,
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
            Self::UnknownSeam(seam) => write!(formatter, "unknown seam {}", seam.0),
            Self::SegmentEndpointOffBoundary => formatter.write_str(
                "both segment endpoints must lie on the selected panel boundary",
            ),
            Self::SegmentDoesNotSplitPanel => formatter.write_str(
                "segment does not split the selected convex panel into two valid panels",
            ),
            Self::CannotFoldCut(seam) => {
                write!(formatter, "seam {} is a cut and cannot be folded", seam.0)
            }
            Self::NonFiniteFoldAngle => formatter.write_str("fold angle must be finite"),
            Self::TooManyRenderVertices => {
                formatter.write_str("render snapshot exceeds u32 index capacity")
            }
        }
    }
}

impl std::error::Error for ModelError {}

impl PaperModel {
    pub fn rectangle(width: f32, height: f32) -> Result<Self, ModelError> {
        if !width.is_finite() || !height.is_finite() || width <= 0.0 || height <= 0.0 {
            return Err(ModelError::InvalidSheetDimensions);
        }
        let half_width = width * 0.5;
        let half_height = height * 0.5;
        Ok(Self {
            vertices: vec![
                Point2::new(-half_width, -half_height),
                Point2::new(half_width, -half_height),
                Point2::new(half_width, half_height),
                Point2::new(-half_width, half_height),
            ],
            panels: vec![Panel {
                id: PanelId(0),
                vertices: vec![0, 1, 2, 3],
            }],
            seams: Vec::new(),
            next_panel_id: 1,
            next_seam_id: 0,
        })
    }

    pub fn panels(&self) -> &[Panel] {
        &self.panels
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
                    let neighbor = if seam.panel_a == current {
                        Some(seam.panel_b)
                    } else if seam.panel_b == current {
                        Some(seam.panel_a)
                    } else {
                        None
                    };
                    if let Some(neighbor) = neighbor {
                        if visited.insert(neighbor) {
                            queue.push_back(neighbor);
                        }
                    }
                }
            }
        }
        count
    }

    /// Splits a convex panel using a straight boundary-to-boundary segment.
    ///
    /// This first topology primitive is deliberately narrow. Future constrained
    /// triangulation may subdivide arbitrary faces without changing the cut/crease
    /// domain contract established here.
    pub fn split_panel_with_segment(
        &mut self,
        panel_id: PanelId,
        start: Point2,
        end: Point2,
        kind: OperationKind,
    ) -> Result<SeamId, ModelError> {
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
        let polygon: Vec<Point2> = self.panels[panel_index]
            .vertices
            .iter()
            .map(|&vertex| self.vertices[vertex])
            .collect();
        if !point_on_boundary(start, &polygon) || !point_on_boundary(end, &polygon) {
            return Err(ModelError::SegmentEndpointOffBoundary);
        }
        let (a_polygon, b_polygon) = split_convex_polygon(&polygon, start, end)
            .ok_or(ModelError::SegmentDoesNotSplitPanel)?;

        let original_id = self.panels[panel_index].id;
        let new_id = PanelId(self.next_panel_id);
        self.next_panel_id += 1;
        let a_vertices = self.intern_polygon(&a_polygon);
        let b_vertices = self.intern_polygon(&b_polygon);
        self.panels[panel_index] = Panel {
            id: original_id,
            vertices: a_vertices,
        };
        self.panels.push(Panel {
            id: new_id,
            vertices: b_vertices,
        });

        let seam_id = SeamId(self.next_seam_id);
        self.next_seam_id += 1;
        self.seams.push(Seam {
            id: seam_id,
            kind,
            start,
            end,
            panel_a: original_id,
            panel_b: new_id,
        });
        Ok(seam_id)
    }

    pub fn render_snapshot(
        &self,
        fold: Option<FoldRequest>,
    ) -> Result<RenderSnapshot, ModelError> {
        let fold_state = match fold {
            Some(request) => Some(self.resolve_fold(request)?),
            None => None,
        };
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        for panel in &self.panels {
            let panel_start =
                u32::try_from(vertices.len()).map_err(|_| ModelError::TooManyRenderVertices)?;
            let should_rotate = fold_state
                .as_ref()
                .is_some_and(|state| state.moving_panels.contains(&panel.id));
            for &vertex_index in &panel.vertices {
                let point = self.vertices[vertex_index];
                let mut point_3d = Vec3::new(point.x, point.y, 0.0);
                if should_rotate {
                    let state = fold_state.as_ref().expect("fold state checked above");
                    point_3d = rotate_around_axis(
                        point_3d,
                        state.axis_start,
                        state.axis_end,
                        state.angle_radians,
                    );
                }
                vertices.push([point_3d.x, point_3d.y, point_3d.z]);
            }
            for triangle_offset in 1..panel.vertices.len().saturating_sub(1) {
                let b = u32::try_from(triangle_offset)
                    .map_err(|_| ModelError::TooManyRenderVertices)?;
                let c = u32::try_from(triangle_offset + 1)
                    .map_err(|_| ModelError::TooManyRenderVertices)?;
                indices.extend_from_slice(&[panel_start, panel_start + b, panel_start + c]);
            }
        }

        let seams = self
            .seams
            .iter()
            .map(|seam| RenderSeam {
                kind: seam.kind,
                start: [seam.start.x, seam.start.y, 0.0],
                end: [seam.end.x, seam.end.y, 0.0],
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
        let seam = self
            .seams
            .iter()
            .find(|seam| seam.id == request.seam)
            .ok_or(ModelError::UnknownSeam(request.seam))?;
        if seam.kind == OperationKind::Cut {
            return Err(ModelError::CannotFoldCut(request.seam));
        }
        let moving_panels = self.connected_panels_without_seam(seam.panel_b, seam.id);
        Ok(ResolvedFold {
            axis_start: Vec3::new(seam.start.x, seam.start.y, 0.0),
            axis_end: Vec3::new(seam.end.x, seam.end.y, 0.0),
            angle_radians: request.angle_radians,
            moving_panels,
        })
    }

    fn connected_panels_without_seam(&self, start: PanelId, excluded: SeamId) -> HashSet<PanelId> {
        let mut visited = HashSet::from([start]);
        let mut queue = VecDeque::from([start]);
        while let Some(current) = queue.pop_front() {
            for seam in self
                .seams
                .iter()
                .filter(|seam| seam.kind == OperationKind::Crease && seam.id != excluded)
            {
                let neighbor = if seam.panel_a == current {
                    Some(seam.panel_b)
                } else if seam.panel_b == current {
                    Some(seam.panel_a)
                } else {
                    None
                };
                if let Some(neighbor) = neighbor {
                    if visited.insert(neighbor) {
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        visited
    }

    fn intern_polygon(&mut self, polygon: &[Point2]) -> Vec<usize> {
        polygon
            .iter()
            .copied()
            .map(|point| self.intern_vertex(point))
            .collect()
    }

    fn intern_vertex(&mut self, point: Point2) -> usize {
        if let Some((index, _)) = self
            .vertices
            .iter()
            .enumerate()
            .find(|(_, candidate)| squared_distance(**candidate, point) <= EPSILON * EPSILON)
        {
            return index;
        }
        self.vertices.push(point);
        self.vertices.len() - 1
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
    axis_start: Vec3,
    axis_end: Vec3,
    angle_radians: f32,
    moving_panels: HashSet<PanelId>,
}

fn split_convex_polygon(
    polygon: &[Point2],
    start: Point2,
    end: Point2,
) -> Option<(Vec<Point2>, Vec<Point2>)> {
    let direction = Point2::new(end.x - start.x, end.y - start.y);
    let mut positive = Vec::new();
    let mut negative = Vec::new();

    for index in 0..polygon.len() {
        let current = polygon[index];
        let next = polygon[(index + 1) % polygon.len()];
        let current_side = signed_side(direction, start, current);
        let next_side = signed_side(direction, start, next);

        if current_side >= -EPSILON {
            push_distinct(&mut positive, current);
        }
        if current_side <= EPSILON {
            push_distinct(&mut negative, current);
        }
        if (current_side > EPSILON && next_side < -EPSILON)
            || (current_side < -EPSILON && next_side > EPSILON)
        {
            let t = current_side / (current_side - next_side);
            let intersection = Point2::new(
                current.x + (next.x - current.x) * t,
                current.y + (next.y - current.y) * t,
            );
            push_distinct(&mut positive, intersection);
            push_distinct(&mut negative, intersection);
        }
    }

    normalize_polygon(&mut positive);
    normalize_polygon(&mut negative);
    (positive.len() >= 3 && negative.len() >= 3).then_some((positive, negative))
}

fn point_on_boundary(point: Point2, polygon: &[Point2]) -> bool {
    (0..polygon.len()).any(|index| {
        point_on_segment(
            point,
            polygon[index],
            polygon[(index + 1) % polygon.len()],
        )
    })
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

fn signed_side(direction: Point2, origin: Point2, point: Point2) -> f32 {
    direction.x * (point.y - origin.y) - direction.y * (point.x - origin.x)
}

fn push_distinct(points: &mut Vec<Point2>, point: Point2) {
    if points
        .last()
        .is_none_or(|last| squared_distance(*last, point) > EPSILON * EPSILON)
    {
        points.push(point);
    }
}

fn normalize_polygon(points: &mut Vec<Point2>) {
    if points.len() > 1
        && squared_distance(
            points[0],
            *points.last().expect("non-empty polygon"),
        ) <= EPSILON * EPSILON
    {
        points.pop();
    }
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

    fn split(kind: OperationKind) -> (PaperModel, SeamId) {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        let seam = model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -0.5),
                Point2::new(0.0, 0.5),
                kind,
            )
            .unwrap();
        (model, seam)
    }

    #[test]
    fn crease_splits_topology_but_preserves_connectivity() {
        let (model, _) = split(OperationKind::Crease);
        assert_eq!(model.panels().len(), 2);
        assert_eq!(model.component_count(), 1);
    }

    #[test]
    fn cut_splits_connectivity() {
        let (model, seam) = split(OperationKind::Cut);
        assert_eq!(model.panels().len(), 2);
        assert_eq!(model.component_count(), 2);
        assert!(matches!(
            model.render_snapshot(Some(FoldRequest {
                seam,
                angle_radians: 0.5
            })),
            Err(ModelError::CannotFoldCut(_))
        ));
    }

    #[test]
    fn crease_fold_produces_depth_and_three_d_lab_mesh() {
        let (model, seam) = split(OperationKind::Crease);
        let fold = FoldRequest {
            seam,
            angle_radians: std::f32::consts::FRAC_PI_2,
        };
        let snapshot = model.render_snapshot(Some(fold)).unwrap();
        assert!(snapshot
            .vertices
            .iter()
            .any(|vertex| vertex[2].abs() > 0.5));
        assert_eq!(snapshot.indices.len(), 12);
        assert_eq!(model.three_d_mesh(Some(fold)).unwrap().triangle_count(), 4);
    }

    #[test]
    fn split_rejects_non_boundary_endpoints() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        assert_eq!(
            model.split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, 0.0),
                Point2::new(0.0, 0.5),
                OperationKind::Cut,
            ),
            Err(ModelError::SegmentEndpointOffBoundary)
        );
    }
}
