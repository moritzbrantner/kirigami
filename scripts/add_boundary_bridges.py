from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise SystemExit(f"{label}: pattern not found")
    return text.replace(old, new, 1)


# --- topology --------------------------------------------------------------------
topology_path = Path("crates/kirigami-core/src/topology.rs")
topology = topology_path.read_text()

topology = replace_once(
    topology,
    '''    PolylineOverlapsBoundary,
    PolylineCrossingAmbiguous,
    SegmentDoesNotSplitFace,''',
    '''    PolylineOverlapsBoundary,
    PolylineCrossingAmbiguous,
    BoundaryBridgeRequiresDistinctComponents,
    SegmentDoesNotSplitFace,''',
    "bridge error enum",
)

topology = replace_once(
    topology,
    '''            Self::PolylineCrossingAmbiguous => formatter
                .write_str("polyline touches a face boundary without crossing into another face"),
            Self::SegmentDoesNotSplitFace => {''',
    '''            Self::PolylineCrossingAmbiguous => formatter
                .write_str("polyline touches a face boundary without crossing into another face"),
            Self::BoundaryBridgeRequiresDistinctComponents => formatter
                .write_str("boundary bridge endpoints must lie on distinct boundary components"),
            Self::SegmentDoesNotSplitFace => {''',
    "bridge error display",
)

topology = replace_once(
    topology,
    '''    pub fn half_edge_count(&self) -> usize {
        self.half_edges.len()
    }

    pub fn face_polygon(&self, face: FaceId) -> Result<Vec<Point2>, TopologyError> {''',
    '''    pub fn half_edge_count(&self) -> usize {
        self.half_edges.len()
    }

    pub fn boundary_component_count(&self, face: FaceId) -> Result<usize, TopologyError> {
        let face = self.face(face)?;
        Ok(face.holes.len() + 1)
    }

    pub fn face_polygon(&self, face: FaceId) -> Result<Vec<Point2>, TopologyError> {''',
    "boundary component count",
)

topology = replace_once(
    topology,
    '''    pub fn triangulate_face(&self, face: FaceId) -> Result<FaceTriangulation, TopologyError> {
        let boundaries = self.face_boundary_loops(face)?;
        if boundaries.holes.is_empty() {
            return triangulate_simple_polygon(face, boundaries.outer);
        }

        let mut outer = boundaries.outer;''',
    '''    pub fn triangulate_face(&self, face: FaceId) -> Result<FaceTriangulation, TopologyError> {
        let face_data = self.face(face)?;
        let mut has_bridge = self.cycle_has_same_face_twin(face_data.outer, face)?;
        for hole in &face_data.holes {
            has_bridge |= self.cycle_has_same_face_twin(*hole, face)?;
        }
        let boundaries = self.face_boundary_loops(face)?;
        if boundaries.holes.is_empty() && !has_bridge {
            return triangulate_simple_polygon(face, boundaries.outer);
        }

        let mut outer = boundaries.outer;''',
    "bridge-aware triangulation",
)

bridge_methods = r'''
    /// Connects two distinct boundary components of one face with an open path.
    /// No new face is created: an outer-to-hole bridge opens an annulus, while a
    /// hole-to-hole bridge merges two inner boundary components. The edit is transactional.
    pub fn bridge_boundary_components_with_polyline(
        &mut self,
        face: FaceId,
        points: &[Point2],
    ) -> Result<(), TopologyError> {
        let mut candidate = self.clone();
        candidate.bridge_boundary_components_with_polyline_in_place(face, points)?;
        *self = candidate;
        Ok(())
    }

    fn bridge_boundary_components_with_polyline_in_place(
        &mut self,
        face: FaceId,
        points: &[Point2],
    ) -> Result<(), TopologyError> {
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

        let start = points[0];
        let end = *points.last().expect("path length checked");
        let start_root = self.boundary_component_root_for_point(face, start)?;
        let end_root = self.boundary_component_root_for_point(face, end)?;
        if start_root == end_root {
            return Err(TopologyError::BoundaryBridgeRequiresDistinctComponents);
        }

        if points[1..points.len() - 1]
            .iter()
            .any(|point| !self.face_contains_point_strict(face, *point).unwrap_or(false))
        {
            return Err(TopologyError::PolylineLeavesFace);
        }
        for segment in points.windows(2) {
            if !self.face_contains_point_strict(face, interpolate(segment[0], segment[1], 0.5))? {
                return Err(TopologyError::PolylineLeavesFace);
            }
            for boundary in self.face_boundary_half_edges(face)? {
                for edge_id in boundary {
                    let edge = self.edge(edge_id);
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
                        BoundaryIntersection::Point { point, .. }
                            if approximately_equal(point, start)
                                || approximately_equal(point, end) => {}
                        BoundaryIntersection::Point { .. } => {
                            return Err(TopologyError::PolylineLeavesFace);
                        }
                    }
                }
            }
        }

        let (start_vertex, start_out) =
            self.locate_or_insert_boundary_vertex_on_cycle(face, start_root, start)?;
        let (end_vertex, end_out) =
            self.locate_or_insert_boundary_vertex_on_cycle(face, end_root, end)?;
        if start_vertex == end_vertex {
            return Err(TopologyError::DegenerateSegment);
        }

        let start_prev = self.edge(start_out).prev;
        let end_prev = self.edge(end_out).prev;
        let segment_count = points.len() - 1;
        let edge_base = self.half_edges.len();
        let mut forward = Vec::with_capacity(segment_count);
        let mut reverse = Vec::with_capacity(segment_count);
        for index in 0..segment_count {
            forward.push(HalfEdgeId(
                u32::try_from(edge_base + index * 2).map_err(|_| TopologyError::TooManyVertices)?,
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
                face,
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
        self.edge_mut(end_out).prev = *forward.last().expect("non-empty bridge");
        self.edge_mut(end_prev).next = *reverse.last().expect("non-empty bridge");
        self.edge_mut(start_out).prev = reverse[0];

        let outer = self.face(face)?.outer;
        if start_root == outer || end_root == outer {
            let removed_hole = if start_root == outer { end_root } else { start_root };
            self.face_mut(face).holes.retain(|root| *root != removed_hole);
        } else {
            self.face_mut(face).holes.retain(|root| *root != end_root);
        }

        self.validate()?;
        Ok(())
    }

'''
if "bridge_boundary_components_with_polyline" not in topology:
    marker = "    pub fn split_face_with_segment(\n"
    if marker not in topology:
        raise SystemExit("bridge method insertion marker not found")
    topology = topology.replace(marker, bridge_methods + marker, 1)

validate_old = '''            let loops = self.face_boundary_loops(face)?;
            if !valid_face_loop(&loops.outer) {
                return Err(TopologyError::InvalidTopology);
            }
            for hole in &loops.holes {
                if !valid_face_loop(hole)
                    || point_on_polygon_boundary(hole[0], &loops.outer)
                    || !point_in_polygon(hole[0], &loops.outer)
                    || polygons_intersect(hole, &loops.outer)
                {
                    return Err(TopologyError::InvalidTopology);
                }
            }'''
validate_new = '''            let face_data = self.face(face)?;
            self.validate_boundary_cycle(face_data.outer, face)?;
            for hole_root in &face_data.holes {
                self.validate_boundary_cycle(*hole_root, face)?;
            }
            let loops = self.face_boundary_loops(face)?;
            for hole in &loops.holes {
                if point_on_polygon_boundary(hole[0], &loops.outer)
                    || !point_in_polygon(hole[0], &loops.outer)
                    || polygons_intersect(hole, &loops.outer)
                {
                    return Err(TopologyError::InvalidTopology);
                }
            }'''
topology = replace_once(topology, validate_old, validate_new, "boundary walk validation")

helper_methods = r'''
    fn boundary_component_root_for_point(
        &self,
        face: FaceId,
        point: Point2,
    ) -> Result<HalfEdgeId, TopologyError> {
        let face_data = self.face(face)?;
        let mut found = None;
        for root in std::iter::once(face_data.outer).chain(face_data.holes.iter().copied()) {
            let cycle = self.cycle_half_edges(root, face)?;
            let on_component = cycle.iter().any(|edge_id| {
                let edge = self.edge(*edge_id);
                let start = self.vertex(edge.origin).point;
                let end = self.vertex(self.edge(edge.next).origin).point;
                point_on_segment(point, start, end)
            });
            if on_component {
                if found.is_some() {
                    return Err(TopologyError::InvalidTopology);
                }
                found = Some(root);
            }
        }
        found.ok_or(TopologyError::PointOffBoundary)
    }

    fn locate_or_insert_boundary_vertex_on_cycle(
        &mut self,
        face: FaceId,
        boundary: HalfEdgeId,
        point: Point2,
    ) -> Result<(VertexId, HalfEdgeId), TopologyError> {
        let cycle = self.cycle_half_edges(boundary, face)?;
        for edge_id in &cycle {
            let origin = self.edge(*edge_id).origin;
            if approximately_equal(self.vertex(origin).point, point) {
                return Ok((origin, *edge_id));
            }
        }
        for edge_id in cycle {
            let edge = self.edge(edge_id);
            let start = self.vertex(edge.origin).point;
            let end = self.vertex(self.edge(edge.next).origin).point;
            if point_on_segment(point, start, end) {
                let vertex = self.split_half_edge(edge_id, point)?;
                let outgoing = self
                    .cycle_half_edges(boundary, face)?
                    .into_iter()
                    .find(|candidate| self.edge(*candidate).origin == vertex)
                    .ok_or(TopologyError::InvalidTopology)?;
                return Ok((vertex, outgoing));
            }
        }
        Err(TopologyError::PointOffBoundary)
    }

    fn cycle_has_same_face_twin(
        &self,
        boundary: HalfEdgeId,
        face: FaceId,
    ) -> Result<bool, TopologyError> {
        Ok(self.cycle_half_edges(boundary, face)?.into_iter().any(|edge_id| {
            self.edge(edge_id)
                .twin
                .is_some_and(|twin| self.edge(twin).face == face)
        }))
    }

    fn validate_boundary_cycle(
        &self,
        boundary: HalfEdgeId,
        face: FaceId,
    ) -> Result<(), TopologyError> {
        let edges = self.cycle_half_edges(boundary, face)?;
        if edges.len() < 3 {
            return Err(TopologyError::InvalidTopology);
        }
        let points: Vec<Point2> = edges
            .iter()
            .map(|edge_id| self.vertex(self.edge(*edge_id).origin).point)
            .collect();
        if signed_area(&points).abs() <= polygon_area_tolerance(&points) {
            return Err(TopologyError::InvalidTopology);
        }

        for first in 0..edges.len() {
            let first_id = edges[first];
            let first_edge = self.edge(first_id);
            let first_start_vertex = first_edge.origin;
            let first_end_vertex = self.edge(first_edge.next).origin;
            let first_start = self.vertex(first_start_vertex).point;
            let first_end = self.vertex(first_end_vertex).point;
            if approximately_equal(first_start, first_end) {
                return Err(TopologyError::InvalidTopology);
            }

            for second in (first + 1)..edges.len() {
                let adjacent = second == first + 1 || (first == 0 && second + 1 == edges.len());
                if adjacent {
                    continue;
                }
                let second_id = edges[second];
                let second_edge = self.edge(second_id);
                let second_start_vertex = second_edge.origin;
                let second_end_vertex = self.edge(second_edge.next).origin;
                let second_start = self.vertex(second_start_vertex).point;
                let second_end = self.vertex(second_end_vertex).point;
                if first_edge.twin == Some(second_id) || second_edge.twin == Some(first_id) {
                    continue;
                }

                let shares_vertex = first_start_vertex == second_start_vertex
                    || first_start_vertex == second_end_vertex
                    || first_end_vertex == second_start_vertex
                    || first_end_vertex == second_end_vertex;
                match segment_boundary_intersection(
                    first_start,
                    first_end,
                    second_start,
                    second_end,
                ) {
                    BoundaryIntersection::None => {}
                    BoundaryIntersection::Point { .. } if shares_vertex => {}
                    BoundaryIntersection::Point { .. } | BoundaryIntersection::Overlap => {
                        return Err(TopologyError::InvalidTopology);
                    }
                }
            }
        }
        Ok(())
    }

'''
if "fn boundary_component_root_for_point" not in topology:
    marker = "    fn locate_or_insert_boundary_vertex(\n"
    if marker not in topology:
        raise SystemExit("boundary helper insertion marker not found")
    topology = topology.replace(marker, helper_methods + marker, 1)

if "fn outer_to_hole_bridge_opens_annulus" not in topology:
    marker = '''    #[test]
    fn traces_path_across_existing_faces() {'''
    tests = r'''    #[test]
    fn outer_to_hole_bridge_opens_annulus() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-2.0, -2.0),
            Point2::new(2.0, -2.0),
            Point2::new(2.0, 2.0),
            Point2::new(-2.0, 2.0),
        ])
        .unwrap();
        topology
            .insert_closed_loop(
                FaceId(0),
                &[
                    Point2::new(-0.5, -0.5),
                    Point2::new(0.5, -0.5),
                    Point2::new(0.5, 0.5),
                    Point2::new(-0.5, 0.5),
                ],
            )
            .unwrap();

        topology
            .bridge_boundary_components_with_polyline(
                FaceId(0),
                &[Point2::new(-2.0, 0.0), Point2::new(-0.5, 0.0)],
            )
            .unwrap();

        assert_eq!(topology.boundary_component_count(FaceId(0)).unwrap(), 1);
        assert!(topology.face_boundary_loops(FaceId(0)).unwrap().holes.is_empty());
        topology.validate().unwrap();
        let triangulation = topology.triangulate_face(FaceId(0)).unwrap();
        let area: f32 = triangulation
            .triangles
            .iter()
            .map(|triangle| {
                triangle_area(
                    triangulation.vertices[triangle[0] as usize],
                    triangulation.vertices[triangle[1] as usize],
                    triangulation.vertices[triangle[2] as usize],
                )
            })
            .sum();
        assert!((area - 15.0).abs() < 1.0e-4);
    }

    #[test]
    fn hole_to_hole_bridge_merges_inner_components() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-3.0, -2.0),
            Point2::new(3.0, -2.0),
            Point2::new(3.0, 2.0),
            Point2::new(-3.0, 2.0),
        ])
        .unwrap();
        topology
            .insert_closed_loop(
                FaceId(0),
                &[
                    Point2::new(-1.75, -0.5),
                    Point2::new(-0.75, -0.5),
                    Point2::new(-0.75, 0.5),
                    Point2::new(-1.75, 0.5),
                ],
            )
            .unwrap();
        topology
            .insert_closed_loop(
                FaceId(0),
                &[
                    Point2::new(0.75, -0.5),
                    Point2::new(1.75, -0.5),
                    Point2::new(1.75, 0.5),
                    Point2::new(0.75, 0.5),
                ],
            )
            .unwrap();

        topology
            .bridge_boundary_components_with_polyline(
                FaceId(0),
                &[Point2::new(-0.75, 0.0), Point2::new(0.75, 0.0)],
            )
            .unwrap();

        assert_eq!(topology.boundary_component_count(FaceId(0)).unwrap(), 2);
        assert_eq!(topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(), 1);
        topology.validate().unwrap();
        assert!(topology.triangulate_face(FaceId(0)).is_ok());
    }

    #[test]
    fn boundary_bridge_rejects_same_component_transactionally() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-2.0, -2.0),
            Point2::new(2.0, -2.0),
            Point2::new(2.0, 2.0),
            Point2::new(-2.0, 2.0),
        ])
        .unwrap();
        let before = topology.clone();
        assert_eq!(
            topology.bridge_boundary_components_with_polyline(
                FaceId(0),
                &[Point2::new(-2.0, 0.0), Point2::new(2.0, 0.0)],
            ),
            Err(TopologyError::BoundaryBridgeRequiresDistinctComponents)
        );
        assert_eq!(topology, before);
    }

    #[test]
    fn traces_path_across_existing_faces() {'''
    if marker not in topology:
        raise SystemExit("topology bridge test marker not found")
    topology = topology.replace(marker, tests, 1)

topology_path.write_text(topology)


# --- domain ----------------------------------------------------------------------
lib_path = Path("crates/kirigami-core/src/lib.rs")
lib = lib_path.read_text()

lib = replace_once(
    lib,
    '''pub struct Seam {
    pub id: SeamId,
    pub operation: OperationId,
    pub kind: OperationKind,
    pub start: Point2,
    pub end: Point2,
    pub panel_a: PanelId,
    pub panel_b: PanelId,
}''',
    '''pub struct Seam {
    pub id: SeamId,
    pub operation: OperationId,
    pub kind: OperationKind,
    pub start: Point2,
    pub end: Point2,
    /// Creases connect two panels. A cut boundary bridge can have the same panel on
    /// both sides because it opens material without creating a second panel.
    pub panel_a: PanelId,
    pub panel_b: PanelId,
}''',
    "seam bridge semantics",
)

bridge_domain = r'''
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

'''
if "pub fn cut_boundary_bridge" not in lib:
    marker = "    fn split_panel_with_path(\n"
    if marker not in lib:
        raise SystemExit("domain bridge insertion marker not found")
    lib = lib.replace(marker, bridge_domain + marker, 1)

lib = replace_once(
    lib,
    '''        | TopologyError::PolylineOverlapsBoundary
        | TopologyError::PolylineCrossingAmbiguous
        | TopologyError::SegmentDoesNotSplitFace''',
    '''        | TopologyError::PolylineOverlapsBoundary
        | TopologyError::PolylineCrossingAmbiguous
        | TopologyError::BoundaryBridgeRequiresDistinctComponents
        | TopologyError::SegmentDoesNotSplitFace''',
    "bridge error mapping",
)

if "fn boundary_bridge_cut_opens_annulus_without_new_panel" not in lib:
    marker = '''    #[test]
    fn second_disjoint_closed_cut_adds_second_hole() {'''
    tests = r'''    #[test]
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
            model.topology().boundary_component_count(FaceId(0)).unwrap(),
            1
        );
        let bridge_segments: Vec<&Seam> = model
            .seams()
            .iter()
            .filter(|seam| seam.operation == bridge)
            .collect();
        assert_eq!(bridge_segments.len(), 1);
        assert!(bridge_segments
            .iter()
            .all(|seam| seam.panel_a == PanelId(0) && seam.panel_b == PanelId(0)));
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
        assert_eq!(model.topology().boundary_component_count(FaceId(0)).unwrap(), 2);
        assert_eq!(model.component_count(), 3);
        assert!(model.render_snapshot(None).is_ok());
    }

    #[test]
    fn second_disjoint_closed_cut_adds_second_hole() {'''
    if marker not in lib:
        raise SystemExit("domain bridge test marker not found")
    lib = lib.replace(marker, tests, 1)

lib_path.write_text(lib)


# --- wasm/browser proof -----------------------------------------------------------
wasm_path = Path("crates/kirigami-wasm/src/lib.rs")
wasm = wasm_path.read_text()
wasm = replace_once(
    wasm,
    '''        "hole" => {
            model
                .cut_closed_path(
                    PanelId(0),
                    &[
                        Point2::new(-0.45, -0.35),
                        Point2::new(0.45, -0.35),
                        Point2::new(0.45, 0.35),
                        Point2::new(-0.45, 0.35),
                    ],
                )
                .map_err(js_error)?;
            None
        }
        _ => return Err(JsValue::from_str("mode must be 'crease', 'cut', or 'hole'")),''',
    '''        "hole" => {
            model
                .cut_closed_path(
                    PanelId(0),
                    &[
                        Point2::new(-0.45, -0.35),
                        Point2::new(0.45, -0.35),
                        Point2::new(0.45, 0.35),
                        Point2::new(-0.45, 0.35),
                    ],
                )
                .map_err(js_error)?;
            None
        }
        "bridge" => {
            model
                .cut_closed_path(
                    PanelId(0),
                    &[
                        Point2::new(-0.45, -0.35),
                        Point2::new(0.45, -0.35),
                        Point2::new(0.45, 0.35),
                        Point2::new(-0.45, 0.35),
                    ],
                )
                .map_err(js_error)?;
            model
                .cut_boundary_bridge(
                    PanelId(0),
                    &[Point2::new(-1.2, 0.0), Point2::new(-0.45, 0.0)],
                )
                .map_err(js_error)?;
            None
        }
        _ => return Err(JsValue::from_str(
            "mode must be 'crease', 'cut', 'hole', or 'bridge'",
        )),''',
    "wasm bridge mode",
)
wasm_path.write_text(wasm)

index_path = Path("web/index.html")
index = index_path.read_text()
index = replace_once(
    index,
    '''      <p class="lede">Explore creases, open cuts, and closed cuts that create detached inner paper and true face holes.</p>''',
    '''      <p class="lede">Explore creases, open cuts, closed cuts, and boundary bridges that open annuli or merge hole boundaries.</p>''',
    "browser bridge lede",
)
index = replace_once(
    index,
    '''          <option value="hole">Closed cut / hole</option>''',
    '''          <option value="hole">Closed cut / hole</option>
          <option value="bridge">Boundary bridge</option>''',
    "browser bridge option",
)
index_path.write_text(index)


# --- roadmap / future 3D target approximation -----------------------------------
roadmap_path = Path("ROADMAP.md")
roadmap = roadmap_path.read_text()
roadmap = replace_once(
    roadmap,
    '''- Multiple disjoint or nested hole loops with deterministic hole-aware rendering triangulation.
- Cuts that split connected components.''',
    '''- Multiple disjoint or nested hole loops with deterministic hole-aware rendering triangulation.
- Boundary-bridge cuts that connect outer-to-hole or hole-to-hole components without inventing a new panel.
- Cuts that split connected components.''',
    "roadmap implemented bridge",
)
roadmap = replace_once(
    roadmap,
    '''1. Extend open-path arrangement to boundary bridges involving inner loops when a cut should open an annulus instead of splitting it into two faces.
2. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
3. Upgrade rendering/physics triangulation to constrained Delaunay when mesh quality requires it; the outer/inner loops remain authoritative regardless of triangulator.
4. Add multi-crease fold state and constraint propagation so genuinely bent creases have an authoritative solver. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
5. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
6. Add layer ordering and flat-foldability validation.
7. Add SVG/FOLD import/export and deterministic pattern fixtures.
8. Add inverse-design/optimization experiments only after forward topology and validity are well tested.''',
    '''1. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
2. Upgrade rendering/physics triangulation to constrained Delaunay when mesh quality requires it; the authoritative boundary graph remains independent of the triangulator.
3. Add multi-crease fold state and constraint propagation so genuinely bent creases have an authoritative solver. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
4. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
5. Add layer ordering and flat-foldability validation.
6. Add SVG/FOLD import/export and deterministic pattern fixtures.
7. Add the 3D-target approximation layer described below once forward folding, collisions, and validity can score candidates reliably.
8. Add broader inverse-design/optimization experiments only after the target-approximation loop is deterministic and measurable.''',
    "roadmap next slices",
)
roadmap = replace_once(
    roadmap,
    '''Closed-path/hole arrangement and multiple boundary loops are now part of the deterministic topology foundation. Hole-aware rendering uses Earcut as a replaceable consumer of authoritative loops; the next algorithm work is boundary-bridge arrangement, BVH/self-intersection, then constrained triangulation and multi-crease constraint solving.''',
    '''Closed-path/hole arrangement, multiple boundary loops, and boundary bridges are now part of the deterministic topology foundation. Hole-aware rendering uses Earcut as a replaceable consumer of authoritative boundary geometry; the next algorithm work is BVH/self-intersection, then constrained triangulation and multi-crease constraint solving.''',
    "roadmap algorithm priorities",
)
if "## 3D target approximation" not in roadmap:
    roadmap += r'''

## 3D target approximation

A future user flow should accept an uploaded 3D model and produce a manufacturable kirigami approximation. File-format ingestion is not a `kirigami-core` responsibility: shared asset/3D adapters should normalize OBJ/glTF/GLB or other supported sources into `three_d_core::Mesh`, then the approximation layer consumes that renderer-neutral target mesh.

The approximation layer should remain outside the authoritative paper model. It proposes deterministic candidate command sequences (panels, cuts, creases, and fold targets); `kirigami-core` validates and evaluates those candidates using the same topology and folding rules as hand-authored patterns. The initial objective should balance surface/shape error against panel count, total cut length, crease complexity, fold-angle complexity, self-intersection, flat-foldability/manufacturability, and material bounds. Candidate generation must retain seeds, input fingerprints, objective weights, and evaluation receipts so improvements can be benchmarked over time.

The first useful vertical slice should use a bounded static target mesh, not arbitrary uploaded files: simplify/segment the target into approximately developable patches, generate one deterministic candidate pattern, fold it through the authoritative solver, and report geometric error plus pattern-complexity metrics. Browser upload and richer format support should come only after this mesh-to-pattern seam is proven.
'''
roadmap_path.write_text(roadmap)
