from pathlib import Path


def replace_once(text: str, old: str, new: str, name: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise SystemExit(f"{name} pattern not found")
    return text.replace(old, new, 1)


# --- topology arrangement/tracing -------------------------------------------------
topology_path = Path("crates/kirigami-core/src/topology.rs")
topology = topology_path.read_text()

topology = replace_once(
    topology,
    '''    PolylineSelfIntersecting,
    PolylineLeavesFace,
    SegmentDoesNotSplitFace,''',
    '''    PolylineSelfIntersecting,
    PolylineLeavesFace,
    PolylineLeavesTopology,
    PolylineOverlapsBoundary,
    PolylineCrossingAmbiguous,
    SegmentDoesNotSplitFace,''',
    "topology multi-face errors",
)

topology = replace_once(
    topology,
    '''            Self::PolylineLeavesFace => formatter
                .write_str("polyline must stay inside the selected face except at endpoints"),
            Self::SegmentDoesNotSplitFace => {''',
    '''            Self::PolylineLeavesFace => formatter
                .write_str("polyline must stay inside the selected face except at endpoints"),
            Self::PolylineLeavesTopology => {
                formatter.write_str("polyline leaves the current planar subdivision")
            }
            Self::PolylineOverlapsBoundary => {
                formatter.write_str("polyline must cross face boundaries instead of overlapping them")
            }
            Self::PolylineCrossingAmbiguous => {
                formatter.write_str("polyline touches a face boundary without crossing into another face")
            }
            Self::SegmentDoesNotSplitFace => {''',
    "topology multi-face error display",
)

trace_methods = r'''
    /// Decomposes one simple open polyline into ordered fragments, one per traversed
    /// face. Existing face-boundary intersections are inserted deterministically.
    /// The topology is not mutated by this trace.
    pub(super) fn trace_polyline_across_faces(
        &self,
        points: &[Point2],
    ) -> Result<Vec<Vec<Point2>>, TopologyError> {
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
        if !self.point_on_any_face_boundary(points[0])
            || !self.point_on_any_face_boundary(*points.last().expect("path length checked"))
        {
            return Err(TopologyError::PointOffBoundary);
        }

        let final_position = (points.len() - 1) as f64;
        let mut crossings = vec![
            PathCrossing {
                position: 0.0,
                point: points[0],
            },
            PathCrossing {
                position: final_position,
                point: *points.last().expect("path length checked"),
            },
        ];

        for (segment_index, segment) in points.windows(2).enumerate() {
            for edge_index in 0..self.half_edges.len() {
                let edge_id = HalfEdgeId(edge_index as u32);
                let edge = self.edge(edge_id);
                if edge.twin.is_some_and(|twin| twin.0 < edge_id.0) {
                    continue;
                }
                let edge_start = self.vertex(edge.origin).point;
                let edge_end = self.vertex(self.edge(edge.next).origin).point;
                match segment_boundary_intersection(
                    segment[0],
                    segment[1],
                    edge_start,
                    edge_end,
                ) {
                    BoundaryIntersection::None => {}
                    BoundaryIntersection::Overlap => {
                        return Err(TopologyError::PolylineOverlapsBoundary);
                    }
                    BoundaryIntersection::Point { t, point } => {
                        let position = segment_index as f64 + f64::from(t);
                        if position > PATH_POSITION_TOLERANCE
                            && position < final_position - PATH_POSITION_TOLERANCE
                        {
                            crossings.push(PathCrossing { position, point });
                        }
                    }
                }
            }
        }

        crossings.sort_by(|left, right| {
            left.position
                .partial_cmp(&right.position)
                .expect("finite path positions")
        });
        crossings.dedup_by(|right, left| {
            (right.position - left.position).abs() <= PATH_POSITION_TOLERANCE
                && approximately_equal(right.point, left.point)
        });

        let mut fragments = Vec::with_capacity(crossings.len().saturating_sub(1));
        let mut previous_face = None;
        for crossing_pair in crossings.windows(2) {
            let fragment = polyline_slice(points, crossing_pair[0], crossing_pair[1]);
            let face = self.face_containing_fragment(&fragment)?;
            if previous_face == Some(face) {
                return Err(TopologyError::PolylineCrossingAmbiguous);
            }
            previous_face = Some(face);
            fragments.push(fragment);
        }
        if fragments.is_empty() {
            return Err(TopologyError::SegmentDoesNotSplitFace);
        }
        Ok(fragments)
    }

    /// Resolves the face containing the interior of a traced path fragment. Callers
    /// may use this after earlier fragments have already split the topology.
    pub(super) fn face_containing_fragment(
        &self,
        points: &[Point2],
    ) -> Result<FaceId, TopologyError> {
        let probe = points
            .windows(2)
            .find(|segment| squared_distance(segment[0], segment[1]) > EPSILON * EPSILON)
            .map(|segment| interpolate(segment[0], segment[1], 0.5))
            .ok_or(TopologyError::DegenerateSegment)?;
        self.face_containing_point_strict(probe)
    }

    fn face_containing_point_strict(&self, point: Point2) -> Result<FaceId, TopologyError> {
        let mut found = None;
        for face_index in 0..self.faces.len() {
            let face = FaceId(face_index as u32);
            let polygon = self.face_polygon(face)?;
            if point_on_polygon_boundary(point, &polygon) {
                continue;
            }
            if point_in_polygon(point, &polygon) {
                if found.is_some() {
                    return Err(TopologyError::InvalidTopology);
                }
                found = Some(face);
            }
        }
        found.ok_or(TopologyError::PolylineLeavesTopology)
    }

    fn point_on_any_face_boundary(&self, point: Point2) -> bool {
        self.half_edges.iter().enumerate().any(|(edge_index, edge)| {
            let edge_id = HalfEdgeId(edge_index as u32);
            if edge.twin.is_some_and(|twin| twin.0 < edge_id.0) {
                return false;
            }
            let start = self.vertex(edge.origin).point;
            let end = self.vertex(self.edge(edge.next).origin).point;
            point_on_segment(point, start, end)
        })
    }

'''
if "trace_polyline_across_faces" not in topology:
    marker = "    fn boundary_arc_points(\n"
    if marker not in topology:
        raise SystemExit("topology boundary arc marker not found")
    topology = topology.replace(marker, trace_methods + marker, 1)

trace_helpers = r'''
const PATH_POSITION_TOLERANCE: f64 = 1.0e-7;
const PATH_PARAMETER_TOLERANCE: f32 = f32::EPSILON * 64.0;

#[derive(Debug, Clone, Copy)]
struct PathCrossing {
    position: f64,
    point: Point2,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum BoundaryIntersection {
    None,
    Point { t: f32, point: Point2 },
    Overlap,
}

fn polyline_slice(points: &[Point2], start: PathCrossing, end: PathCrossing) -> Vec<Point2> {
    let mut fragment = vec![start.point];
    for (index, point) in points.iter().copied().enumerate().skip(1) {
        let position = index as f64;
        if position > start.position + PATH_POSITION_TOLERANCE
            && position < end.position - PATH_POSITION_TOLERANCE
        {
            if !approximately_equal(*fragment.last().expect("fragment has start"), point) {
                fragment.push(point);
            }
        }
    }
    if !approximately_equal(*fragment.last().expect("fragment has start"), end.point) {
        fragment.push(end.point);
    }
    fragment
}

fn segment_boundary_intersection(
    start: Point2,
    end: Point2,
    edge_start: Point2,
    edge_end: Point2,
) -> BoundaryIntersection {
    let rx = end.x - start.x;
    let ry = end.y - start.y;
    let sx = edge_end.x - edge_start.x;
    let sy = edge_end.y - edge_start.y;
    let qx = edge_start.x - start.x;
    let qy = edge_start.y - start.y;
    let denominator = cross_components(rx, ry, sx, sy);
    let scale_squared = squared_distance(start, end).max(squared_distance(edge_start, edge_end));
    let denominator_tolerance = scale_squared * f32::EPSILON * 32.0;

    if denominator.abs() <= denominator_tolerance {
        if !is_collinear(start, end, edge_start) || !is_collinear(start, end, edge_end) {
            return BoundaryIntersection::None;
        }
        let length_squared = squared_distance(start, end);
        let t0 = ((edge_start.x - start.x) * rx + (edge_start.y - start.y) * ry)
            / length_squared;
        let t1 = ((edge_end.x - start.x) * rx + (edge_end.y - start.y) * ry)
            / length_squared;
        let overlap_start = t0.min(t1).max(0.0);
        let overlap_end = t0.max(t1).min(1.0);
        if overlap_end < overlap_start - PATH_PARAMETER_TOLERANCE {
            return BoundaryIntersection::None;
        }
        if overlap_end - overlap_start > PATH_PARAMETER_TOLERANCE {
            return BoundaryIntersection::Overlap;
        }
        let t = ((overlap_start + overlap_end) * 0.5).clamp(0.0, 1.0);
        return BoundaryIntersection::Point {
            t,
            point: interpolate(start, end, t),
        };
    }

    let t = cross_components(qx, qy, sx, sy) / denominator;
    let u = cross_components(qx, qy, rx, ry) / denominator;
    if !(-PATH_PARAMETER_TOLERANCE..=1.0 + PATH_PARAMETER_TOLERANCE).contains(&t)
        || !(-PATH_PARAMETER_TOLERANCE..=1.0 + PATH_PARAMETER_TOLERANCE).contains(&u)
    {
        return BoundaryIntersection::None;
    }
    let t = t.clamp(0.0, 1.0);
    BoundaryIntersection::Point {
        t,
        point: interpolate(start, end, t),
    }
}

fn cross_components(ax: f32, ay: f32, bx: f32, by: f32) -> f32 {
    ax * by - ay * bx
}

'''
if "enum BoundaryIntersection" not in topology:
    marker = "fn valid_face_loop(points: &[Point2]) -> bool {"
    if marker not in topology:
        raise SystemExit("valid face loop marker not found")
    topology = topology.replace(marker, trace_helpers + marker, 1)

if "fn traces_path_across_existing_faces" not in topology:
    marker = '''    #[test]
    fn polyline_split_creates_half_edge_chain() {'''
    tests = r'''    #[test]
    fn traces_path_across_existing_faces() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ])
        .unwrap();
        topology
            .split_face_with_segment(
                FaceId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
            )
            .unwrap();

        let fragments = topology
            .trace_polyline_across_faces(&[
                Point2::new(-1.0, 0.0),
                Point2::new(1.0, 0.0),
            ])
            .unwrap();
        assert_eq!(fragments.len(), 2);
        assert!(approximately_equal(
            *fragments[0].last().unwrap(),
            Point2::new(0.0, 0.0)
        ));
        assert!(approximately_equal(
            fragments[1][0],
            Point2::new(0.0, 0.0)
        ));
        assert_ne!(
            topology.face_containing_fragment(&fragments[0]).unwrap(),
            topology.face_containing_fragment(&fragments[1]).unwrap()
        );
    }

    #[test]
    fn trace_rejects_boundary_overlap_without_mutation() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ])
        .unwrap();
        topology
            .split_face_with_segment(
                FaceId(0),
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
            )
            .unwrap();
        let before = topology.clone();

        assert_eq!(
            topology.trace_polyline_across_faces(&[
                Point2::new(0.0, -0.75),
                Point2::new(0.0, 0.75),
            ]),
            Err(TopologyError::PolylineOverlapsBoundary)
        );
        assert_eq!(topology, before);
    }

    #[test]
    fn polyline_split_creates_half_edge_chain() {'''
    if marker not in topology:
        raise SystemExit("topology test insertion marker not found")
    topology = topology.replace(marker, tests, 1)

topology_path.write_text(topology)


# --- domain transaction/operation identity ---------------------------------------
lib_path = Path("crates/kirigami-core/src/lib.rs")
lib = lib_path.read_text()

start = lib.index("    fn split_panel_with_path(")
end = lib.index("    pub fn render_snapshot", start)
replacement = r'''    fn split_panel_with_path(
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
        let a_polygon = self.topology.face_polygon(original_face)?;
        let b_polygon = self.topology.face_polygon(new_face)?;

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

'''
lib = lib[:start] + replacement + lib[end:]

if "fn validate_operation_path" not in lib:
    marker = "fn map_split_error(error: TopologyError) -> ModelError {"
    helper = r'''fn validate_operation_path(path: &[Point2], kind: OperationKind) -> Result<(), ModelError> {
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

'''
    if marker not in lib:
        raise SystemExit("map split error marker not found")
    lib = lib.replace(marker, helper + marker, 1)

map_start = lib.index("fn map_split_error(error: TopologyError) -> ModelError {")
map_end = lib.index("\nfn path_is_straight", map_start)
map_function = r'''fn map_split_error(error: TopologyError) -> ModelError {
    match error {
        TopologyError::PointOffBoundary => ModelError::SegmentEndpointOffBoundary,
        TopologyError::DegenerateSegment => ModelError::DegenerateSegment,
        TopologyError::TooFewPathPoints => ModelError::InvalidPath,
        TopologyError::PolylineSelfIntersecting
        | TopologyError::PolylineLeavesFace
        | TopologyError::PolylineLeavesTopology
        | TopologyError::PolylineOverlapsBoundary
        | TopologyError::PolylineCrossingAmbiguous
        | TopologyError::SegmentDoesNotSplitFace
        | TopologyError::SegmentLeavesFace => ModelError::SegmentDoesNotSplitPanel,
        other => ModelError::Topology(other),
    }
}
'''
lib = lib[:map_start] + map_function + lib[map_end:]

if "fn cut_across_existing_crease_is_one_operation" not in lib:
    marker = '''    #[test]
    fn bent_cut_creates_segmented_operation_and_disconnects() {'''
    tests = r'''    #[test]
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
    fn bent_cut_creates_segmented_operation_and_disconnects() {'''
    if marker not in lib:
        raise SystemExit("domain test insertion marker not found")
    lib = lib.replace(marker, tests, 1)

lib_path.write_text(lib)


# --- roadmap ---------------------------------------------------------------------
roadmap_path = Path("ROADMAP.md")
roadmap = roadmap_path.read_text()
implemented_marker = "- Deterministic ear-clipping triangulation for simple concave faces; rendering no longer assumes convex fan triangulation.\n"
implemented_addition = (
    implemented_marker
    + "- Transactional single-face polyline splitting for bent cuts and segmented straight creases.\n"
    + "- Deterministic multi-face path tracing with inserted boundary intersections; one logical operation spans all generated seam segments.\n"
)
if "Deterministic multi-face path tracing" not in roadmap:
    if implemented_marker not in roadmap:
        raise SystemExit("roadmap implemented marker not found")
    roadmap = roadmap.replace(implemented_marker, implemented_addition, 1)

old_next = '''1. Generalize operations from one chord to arbitrary cut and crease polylines, including multiple face crossings and deterministic intersection insertion.
2. Add holes/multiple boundary loops and upgrade triangulation to constrained Delaunay when mesh quality or physical simulation requires it; keep boundary constraints authoritative regardless of triangulator.
3. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
4. Add multi-crease fold state and constraint propagation. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
5. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
6. Add layer ordering and flat-foldability validation.
7. Add SVG/FOLD import/export and deterministic pattern fixtures.
8. Add inverse-design/optimization experiments only after forward topology and validity are well tested.'''
new_next = '''1. Add closed cut paths / holes and multiple boundary loops; then upgrade triangulation to constrained Delaunay when mesh quality or physical simulation requires it while keeping boundary constraints authoritative.
2. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
3. Add multi-crease fold state and constraint propagation so genuinely bent creases have an authoritative solver. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
4. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
5. Add layer ordering and flat-foldability validation.
6. Add SVG/FOLD import/export and deterministic pattern fixtures.
7. Add inverse-design/optimization experiments only after forward topology and validity are well tested.'''
if old_next in roadmap:
    roadmap = roadmap.replace(old_next, new_next, 1)

old_algorithms = "The immediate algorithm work is robust segment intersection across multiple faces, polyline arrangement insertion, constrained triangulation with holes, connected-components traversal, and BVH/self-intersection. Constraint solving and layer ordering follow once multi-operation topology is stable."
new_algorithms = "The immediate algorithm work is closed-path/hole arrangement, constrained triangulation with multiple boundary loops, BVH/self-intersection, and then multi-crease constraint solving plus layer ordering. Multi-face open-path intersection insertion is now part of the deterministic topology foundation."
if old_algorithms in roadmap:
    roadmap = roadmap.replace(old_algorithms, new_algorithms, 1)

roadmap_path.write_text(roadmap)
