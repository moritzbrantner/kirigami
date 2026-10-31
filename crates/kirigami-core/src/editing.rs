use crate::geometry::{interpolate, path_is_straight, point_on_segment, squared_distance};
use crate::topology::{FaceBoundaryLoops, TopologyError};
use crate::{
    EPSILON, ModelError, Operation, OperationId, OperationKind, Panel, PanelId, PaperModel, Point2,
    Seam, SeamId,
};

impl PaperModel {
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

fn segment_parameter(point: Point2, start: Point2, end: Point2) -> f32 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let length_squared = dx * dx + dy * dy;
    ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
}
