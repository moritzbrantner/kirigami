from pathlib import Path


def replace_once(text: str, old: str, new: str, name: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise SystemExit(f"{name} pattern not found")
    return text.replace(old, new, 1)


topology_path = Path("crates/kirigami-core/src/topology.rs")
topology = topology_path.read_text()

topology = replace_once(
    topology,
    '''    DegenerateSegment,
    SegmentDoesNotSplitFace,''',
    '''    DegenerateSegment,
    TooFewPathPoints,
    PolylineSelfIntersecting,
    PolylineLeavesFace,
    SegmentDoesNotSplitFace,''',
    "topology errors",
)
topology = replace_once(
    topology,
    '''            Self::DegenerateSegment => formatter.write_str("segment endpoints must be distinct"),
            Self::SegmentDoesNotSplitFace => {''',
    '''            Self::DegenerateSegment => formatter.write_str("segment endpoints must be distinct"),
            Self::TooFewPathPoints => formatter.write_str("a path requires at least two points"),
            Self::PolylineSelfIntersecting => {
                formatter.write_str("polyline must not self-intersect")
            }
            Self::PolylineLeavesFace => {
                formatter.write_str("polyline must stay inside the selected face except at endpoints")
            }
            Self::SegmentDoesNotSplitFace => {''',
    "topology error display",
)

polyline_impl = r'''
impl PlanarTopology {
    /// Splits one face along an open polyline whose endpoints lie on the face boundary.
    /// Internal path points must stay strictly inside the face. The edit is transactional.
    pub fn split_face_with_polyline(
        &mut self,
        face: FaceId,
        points: &[Point2],
    ) -> Result<FaceId, TopologyError> {
        if points.len() == 2 {
            return self.split_face_with_segment(face, points[0], points[1]);
        }
        let mut candidate = self.clone();
        let new_face = candidate.split_face_with_polyline_in_place(face, points)?;
        *self = candidate;
        Ok(new_face)
    }

    fn split_face_with_polyline_in_place(
        &mut self,
        face: FaceId,
        points: &[Point2],
    ) -> Result<FaceId, TopologyError> {
        if points.len() < 2 {
            return Err(TopologyError::TooFewPathPoints);
        }
        if points.iter().any(|point| !point.is_finite()) {
            return Err(TopologyError::NonFinitePoint);
        }
        if points
            .windows(2)
            .any(|segment| squared_distance(segment[0], segment[1]) <= EPSILON * EPSILON)
        {
            return Err(TopologyError::DegenerateSegment);
        }
        if !polyline_is_simple(points) {
            return Err(TopologyError::PolylineSelfIntersecting);
        }
        self.face(face)?;

        let polygon = self.face_polygon(face)?;
        let start = points[0];
        let end = *points.last().expect("path length checked");
        if !point_on_polygon_boundary(start, &polygon)
            || !point_on_polygon_boundary(end, &polygon)
        {
            return Err(TopologyError::PointOffBoundary);
        }
        if points[1..points.len() - 1].iter().any(|point| {
            point_on_polygon_boundary(*point, &polygon) || !point_in_polygon(*point, &polygon)
        }) {
            return Err(TopologyError::PolylineLeavesFace);
        }

        let start_vertex = self.locate_or_insert_boundary_vertex(face, start)?;
        let end_vertex = self.locate_or_insert_boundary_vertex(face, end)?;
        if start_vertex == end_vertex {
            return Err(TopologyError::DegenerateSegment);
        }

        let boundary = self.face_half_edges(face)?;
        let start_out = boundary
            .iter()
            .copied()
            .find(|edge| self.edge(*edge).origin == start_vertex)
            .ok_or(TopologyError::InvalidTopology)?;
        let end_out = boundary
            .iter()
            .copied()
            .find(|edge| self.edge(*edge).origin == end_vertex)
            .ok_or(TopologyError::InvalidTopology)?;

        let start_to_end_arc = self.boundary_arc_points(start_out, end_vertex)?;
        let end_to_start_arc = self.boundary_arc_points(end_out, start_vertex)?;
        let mut old_loop = start_to_end_arc;
        old_loop.extend(points[1..points.len() - 1].iter().rev().copied());
        let mut new_loop = end_to_start_arc;
        new_loop.extend(points[1..points.len() - 1].iter().copied());
        if !valid_face_loop(&old_loop) || !valid_face_loop(&new_loop) {
            return Err(TopologyError::PolylineLeavesFace);
        }

        let start_prev = self.edge(start_out).prev;
        let end_prev = self.edge(end_out).prev;
        let new_face =
            FaceId(u32::try_from(self.faces.len()).map_err(|_| TopologyError::TooManyVertices)?);
        let segment_count = points.len() - 1;
        let edge_base = self.half_edges.len();
        let mut forward = Vec::with_capacity(segment_count);
        let mut reverse = Vec::with_capacity(segment_count);
        for index in 0..segment_count {
            forward.push(HalfEdgeId(
                u32::try_from(edge_base + index * 2)
                    .map_err(|_| TopologyError::TooManyVertices)?,
            ));
            reverse.push(HalfEdgeId(
                u32::try_from(edge_base + index * 2 + 1)
                    .map_err(|_| TopologyError::TooManyVertices)?,
            ));
        }

        let mut path_vertices = Vec::with_capacity(points.len());
        path_vertices.push(start_vertex);
        for (internal_index, point) in points[1..points.len() - 1].iter().copied().enumerate() {
            let vertex = VertexId(
                u32::try_from(self.vertices.len()).map_err(|_| TopologyError::TooManyVertices)?,
            );
            self.vertices.push(TopologyVertex {
                point,
                outgoing: Some(forward[internal_index + 1]),
            });
            path_vertices.push(vertex);
        }
        path_vertices.push(end_vertex);

        self.faces.push(Face { boundary: end_out });
        for index in 0..segment_count {
            self.half_edges.push(HalfEdge {
                origin: path_vertices[index],
                twin: Some(reverse[index]),
                next: if index + 1 < segment_count {
                    forward[index + 1]
                } else {
                    end_out
                },
                prev: if index == 0 {
                    start_prev
                } else {
                    forward[index - 1]
                },
                face: new_face,
            });
            self.half_edges.push(HalfEdge {
                origin: path_vertices[index + 1],
                twin: Some(forward[index]),
                next: if index == 0 {
                    start_out
                } else {
                    reverse[index - 1]
                },
                prev: if index + 1 == segment_count {
                    end_prev
                } else {
                    reverse[index + 1]
                },
                face,
            });
        }

        self.edge_mut(start_prev).next = forward[0];
        self.edge_mut(end_out).prev = *forward.last().expect("non-empty path");
        self.edge_mut(end_prev).next = *reverse.last().expect("non-empty path");
        self.edge_mut(start_out).prev = reverse[0];
        self.face_mut(face).boundary = start_out;
        self.assign_face_cycle(start_out, face)?;
        self.assign_face_cycle(end_out, new_face)?;
        self.validate()?;
        Ok(new_face)
    }

    fn boundary_arc_points(
        &self,
        start_edge: HalfEdgeId,
        end_vertex: VertexId,
    ) -> Result<Vec<Point2>, TopologyError> {
        let mut points = vec![self.vertex(self.edge(start_edge).origin).point];
        let mut current = start_edge;
        for _ in 0..=self.half_edges.len() {
            current = self.edge(current).next;
            let vertex = self.edge(current).origin;
            points.push(self.vertex(vertex).point);
            if vertex == end_vertex {
                return Ok(points);
            }
        }
        Err(TopologyError::InvalidTopology)
    }
}

fn valid_face_loop(points: &[Point2]) -> bool {
    points.len() >= 3
        && !points
            .windows(2)
            .any(|pair| approximately_equal(pair[0], pair[1]))
        && is_simple_polygon(points)
        && signed_area(points).abs() > polygon_area_tolerance(points)
}

fn polyline_is_simple(points: &[Point2]) -> bool {
    for first in 0..points.len() - 1 {
        for second in (first + 2)..points.len() - 1 {
            if segments_intersect(
                points[first],
                points[first + 1],
                points[second],
                points[second + 1],
            ) {
                return false;
            }
        }
    }
    true
}

'''
if "pub fn split_face_with_polyline" not in topology:
    marker = "fn signed_area(points: &[Point2]) -> f32 {"
    if marker not in topology:
        raise SystemExit("topology helper marker not found")
    topology = topology.replace(marker, polyline_impl + marker, 1)

if "fn polyline_split_creates_half_edge_chain" not in topology:
    marker = '''    #[test]
    fn rejects_chord_that_leaves_concave_face_without_mutation() {'''
    tests = r'''    #[test]
    fn polyline_split_creates_half_edge_chain() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ])
        .unwrap();
        let second = topology
            .split_face_with_polyline(
                FaceId(0),
                &[
                    Point2::new(-1.0, 0.0),
                    Point2::new(0.0, 0.4),
                    Point2::new(1.0, 0.0),
                ],
            )
            .unwrap();

        assert_eq!(second, FaceId(1));
        assert_eq!(topology.face_count(), 2);
        topology.validate().unwrap();
        assert!(topology.triangulate_face(FaceId(0)).is_ok());
        assert!(topology.triangulate_face(FaceId(1)).is_ok());
    }

    #[test]
    fn polyline_split_is_fail_closed_when_path_leaves_face() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ])
        .unwrap();
        let before = topology.clone();
        assert_eq!(
            topology.split_face_with_polyline(
                FaceId(0),
                &[
                    Point2::new(-1.0, 0.0),
                    Point2::new(0.0, 1.5),
                    Point2::new(1.0, 0.0),
                ],
            ),
            Err(TopologyError::PolylineLeavesFace)
        );
        assert_eq!(topology, before);
    }

    #[test]
    fn rejects_self_intersecting_polyline_without_mutation() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-2.0, -2.0),
            Point2::new(2.0, -2.0),
            Point2::new(2.0, 2.0),
            Point2::new(-2.0, 2.0),
        ])
        .unwrap();
        let before = topology.clone();
        assert_eq!(
            topology.split_face_with_polyline(
                FaceId(0),
                &[
                    Point2::new(-2.0, 0.0),
                    Point2::new(1.0, 1.0),
                    Point2::new(-1.0, 1.0),
                    Point2::new(2.0, 0.0),
                ],
            ),
            Err(TopologyError::PolylineSelfIntersecting)
        );
        assert_eq!(topology, before);
    }

    #[test]
    fn rejects_chord_that_leaves_concave_face_without_mutation() {'''
    if marker not in topology:
        raise SystemExit("topology test marker not found")
    topology = topology.replace(marker, tests, 1)

topology_path.write_text(topology)

lib_path = Path("crates/kirigami-core/src/lib.rs")
lib = lib_path.read_text()

lib = replace_once(
    lib,
    '''pub struct Operation {
    pub id: OperationId,
    pub kind: OperationKind,
    pub start: Point2,
    pub end: Point2,
}''',
    '''pub struct Operation {
    pub id: OperationId,
    pub kind: OperationKind,
    pub path: Vec<Point2>,
}''',
    "operation path",
)
lib = replace_once(
    lib,
    '''    DegenerateSegment,
    UnknownPanel(PanelId),''',
    '''    DegenerateSegment,
    InvalidPath,
    BentCreaseRequiresConstraintSolver,
    UnknownPanel(PanelId),''',
    "model errors",
)
lib = replace_once(
    lib,
    '''            Self::DegenerateSegment => formatter.write_str("segment endpoints must be distinct"),
            Self::UnknownPanel(panel) => write!(formatter, "unknown panel {}", panel.0),''',
    '''            Self::DegenerateSegment => formatter.write_str("segment endpoints must be distinct"),
            Self::InvalidPath => formatter.write_str("a path requires at least two finite points"),
            Self::BentCreaseRequiresConstraintSolver => formatter.write_str(
                "a non-straight crease requires the multi-crease constraint solver",
            ),
            Self::UnknownPanel(panel) => write!(formatter, "unknown panel {}", panel.0),''',
    "model error display",
)

start = lib.index("    /// Splits a panel using a straight boundary-to-boundary segment.")
end = lib.index("    pub fn render_snapshot", start)
replacement = r'''    /// Splits a panel using a straight boundary-to-boundary segment.
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

    fn split_panel_with_path(
        &mut self,
        panel_id: PanelId,
        path: &[Point2],
        kind: OperationKind,
    ) -> Result<OperationId, ModelError> {
        if path.len() < 2 || path.iter().any(|point| !point.is_finite()) {
            return Err(ModelError::InvalidPath);
        }
        if path
            .windows(2)
            .any(|segment| squared_distance(segment[0], segment[1]) <= EPSILON * EPSILON)
        {
            return Err(ModelError::DegenerateSegment);
        }
        let panel_index = self
            .panels
            .iter()
            .position(|panel| panel.id == panel_id)
            .ok_or(ModelError::UnknownPanel(panel_id))?;
        let original_face = self.panels[panel_index].face;
        let start = path[0];
        let end = *path.last().expect("path length checked");

        let mut candidate_topology = self.topology.clone();
        let new_face = if path.len() == 2 {
            candidate_topology
                .split_face_with_segment(original_face, start, end)
                .map_err(map_split_error)?
        } else {
            candidate_topology
                .split_face_with_polyline(original_face, path)
                .map_err(map_split_error)?
        };
        let a_polygon = candidate_topology.face_polygon(original_face)?;
        let b_polygon = candidate_topology.face_polygon(new_face)?;

        let original_id = self.panels[panel_index].id;
        let new_id = PanelId(self.next_panel_id);
        let (reattached_seams, mut next_seam_id) = self.reattached_seams_after_split(
            original_id,
            new_id,
            start,
            end,
            &a_polygon,
            &b_polygon,
        )?;

        let operation_id = OperationId(self.next_operation_id);
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

        self.topology = candidate_topology;
        self.panels.push(Panel {
            id: new_id,
            face: new_face,
        });
        self.seams = reattached_seams;
        self.seams.extend(new_seams);
        self.operations.push(Operation {
            id: operation_id,
            kind,
            path: path.to_vec(),
        });
        self.next_panel_id += 1;
        self.next_operation_id += 1;
        self.next_seam_id = next_seam_id;
        Ok(operation_id)
    }

'''
lib = lib[:start] + replacement + lib[end:]

lib = replace_once(
    lib,
    '''        if operation.kind == OperationKind::Cut {
            return Err(ModelError::CannotFoldCut(request.operation));
        }

        let moving_seeds:''',
    '''        if operation.kind == OperationKind::Cut {
            return Err(ModelError::CannotFoldCut(request.operation));
        }
        if !path_is_straight(&operation.path) {
            return Err(ModelError::BentCreaseRequiresConstraintSolver);
        }
        let axis_start = operation.path[0];
        let axis_end = *operation.path.last().expect("operation path validated");

        let moving_seeds:''',
    "fold path guard",
)
lib = replace_once(
    lib,
    '''            axis_start: Vec3::new(operation.start.x, operation.start.y, 0.0),
            axis_end: Vec3::new(operation.end.x, operation.end.y, 0.0),''',
    '''            axis_start: Vec3::new(axis_start.x, axis_start.y, 0.0),
            axis_end: Vec3::new(axis_end.x, axis_end.y, 0.0),''',
    "fold path axis",
)
lib = replace_once(
    lib,
    '''        TopologyError::PointOffBoundary => ModelError::SegmentEndpointOffBoundary,
        TopologyError::DegenerateSegment => ModelError::DegenerateSegment,
        TopologyError::SegmentDoesNotSplitFace | TopologyError::SegmentLeavesFace => {''',
    '''        TopologyError::PointOffBoundary => ModelError::SegmentEndpointOffBoundary,
        TopologyError::DegenerateSegment => ModelError::DegenerateSegment,
        TopologyError::TooFewPathPoints => ModelError::InvalidPath,
        TopologyError::PolylineSelfIntersecting
        | TopologyError::PolylineLeavesFace
        | TopologyError::SegmentDoesNotSplitFace
        | TopologyError::SegmentLeavesFace => {''',
    "split error mapping",
)

path_helper = r'''
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

'''
if "fn path_is_straight" not in lib:
    marker = "fn adjacent_panel(seam: &Seam, panel: PanelId) -> Option<PanelId> {"
    if marker not in lib:
        raise SystemExit("path helper marker not found")
    lib = lib.replace(marker, path_helper + marker, 1)

if "fn bent_cut_creates_segmented_operation" not in lib:
    marker = '''    #[test]
    fn downstream_crease_moves_with_folded_component() {'''
    tests = r'''    #[test]
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
    fn downstream_crease_moves_with_folded_component() {'''
    if marker not in lib:
        raise SystemExit("lib test marker not found")
    lib = lib.replace(marker, tests, 1)

lib_path.write_text(lib)
