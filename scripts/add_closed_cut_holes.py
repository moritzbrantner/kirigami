from pathlib import Path


def replace_once(text: str, old: str, new: str, name: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise SystemExit(f"{name} pattern not found")
    return text.replace(old, new, 1)


def replace_section(text: str, start: str, end: str, replacement: str, name: str) -> str:
    start_index = text.find(start)
    if start_index < 0:
        raise SystemExit(f"{name} start marker not found")
    end_index = text.find(end, start_index)
    if end_index < 0:
        raise SystemExit(f"{name} end marker not found")
    return text[:start_index] + replacement + text[end_index:]


# Dependencies -------------------------------------------------------------------
workspace_path = Path("Cargo.toml")
workspace = workspace_path.read_text()
workspace = replace_once(
    workspace,
    'serde_json = "1"\n',
    'serde_json = "1"\nearcut = "0.4.11"\n',
    "workspace earcut dependency",
)
workspace_path.write_text(workspace)

core_manifest_path = Path("crates/kirigami-core/Cargo.toml")
core_manifest = core_manifest_path.read_text()
core_manifest = replace_once(
    core_manifest,
    '[dependencies]\nserde.workspace = true\n',
    '[dependencies]\nearcut.workspace = true\nserde.workspace = true\n',
    "core earcut dependency",
)
core_manifest_path.write_text(core_manifest)


# Topology -----------------------------------------------------------------------
topology_path = Path("crates/kirigami-core/src/topology.rs")
topology = topology_path.read_text()

topology = replace_once(
    topology,
    'use crate::Point2;\nuse std::fmt;\n',
    'use crate::Point2;\nuse earcut::Earcut;\nuse std::fmt;\n',
    "topology imports",
)

topology = replace_once(
    topology,
    '''#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Face {
    boundary: HalfEdgeId,
}''',
    '''#[derive(Debug, Clone, PartialEq, Eq)]
struct Face {
    outer: HalfEdgeId,
    holes: Vec<HalfEdgeId>,
}''',
    "face boundary loops",
)

topology = replace_once(
    topology,
    '''#[derive(Debug, Clone, PartialEq)]
pub struct FaceTriangulation {
    pub vertices: Vec<Point2>,
    pub triangles: Vec<[u32; 3]>,
}
''',
    '''#[derive(Debug, Clone, PartialEq)]
pub struct FaceTriangulation {
    pub vertices: Vec<Point2>,
    pub triangles: Vec<[u32; 3]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceBoundaryLoops {
    pub outer: Vec<Point2>,
    pub holes: Vec<Vec<Point2>>,
}
''',
    "public face boundary loops",
)

topology = replace_once(
    topology,
    '''            faces: vec![Face {
                boundary: HalfEdgeId(0),
            }],''',
    '''            faces: vec![Face {
                outer: HalfEdgeId(0),
                holes: Vec::new(),
            }],''',
    "initial face",
)

face_methods = r'''    pub fn face_polygon(&self, face: FaceId) -> Result<Vec<Point2>, TopologyError> {
        self.cycle_polygon(self.face(face)?.outer, face)
    }

    pub fn face_boundary_loops(&self, face: FaceId) -> Result<FaceBoundaryLoops, TopologyError> {
        let face_data = self.face(face)?;
        let outer = self.cycle_polygon(face_data.outer, face)?;
        let holes = face_data
            .holes
            .iter()
            .map(|boundary| self.cycle_polygon(*boundary, face))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(FaceBoundaryLoops { outer, holes })
    }

'''
topology = replace_section(
    topology,
    "    pub fn face_polygon(&self, face: FaceId)",
    "    pub fn triangulate_face(&self, face: FaceId)",
    face_methods,
    "face boundary methods",
)

triangulate = r'''    pub fn triangulate_face(&self, face: FaceId) -> Result<FaceTriangulation, TopologyError> {
        let boundaries = self.face_boundary_loops(face)?;
        if boundaries.holes.is_empty() {
            return triangulate_simple_polygon(face, boundaries.outer);
        }

        let mut outer = boundaries.outer;
        if signed_area(&outer) < 0.0 {
            outer.reverse();
        }
        let mut vertices = outer;
        let mut hole_indices = Vec::<u32>::with_capacity(boundaries.holes.len());
        for mut hole in boundaries.holes {
            if signed_area(&hole) > 0.0 {
                hole.reverse();
            }
            hole_indices.push(
                u32::try_from(vertices.len()).map_err(|_| TopologyError::TooManyVertices)?,
            );
            vertices.extend(hole);
        }

        let mut flat_indices = Vec::<u32>::new();
        let mut earcut = Earcut::<f32>::new();
        earcut.earcut::<u32>(
            vertices.iter().map(|point| [point.x, point.y]),
            &hole_indices,
            &mut flat_indices,
        );
        if flat_indices.len() < 3 || flat_indices.len() % 3 != 0 {
            return Err(TopologyError::TriangulationFailed(face));
        }
        let triangles = flat_indices
            .chunks_exact(3)
            .map(|triangle| [triangle[0], triangle[1], triangle[2]])
            .collect();
        Ok(FaceTriangulation {
            vertices,
            triangles,
        })
    }

    /// Inserts a simple closed loop fully inside one face. The original face gains
    /// an inner boundary and the enclosed material becomes a new face. Existing holes
    /// enclosed by the new loop move with the detached inner face.
    pub fn insert_closed_loop(
        &mut self,
        face: FaceId,
        points: &[Point2],
    ) -> Result<FaceId, TopologyError> {
        let mut candidate = self.clone();
        let new_face = candidate.insert_closed_loop_in_place(face, points)?;
        *self = candidate;
        Ok(new_face)
    }

    fn insert_closed_loop_in_place(
        &mut self,
        face: FaceId,
        points: &[Point2],
    ) -> Result<FaceId, TopologyError> {
        self.face(face)?;
        let loop_points = normalize_closed_loop(points)?;
        let boundaries = self.face_boundary_loops(face)?;

        for point in &loop_points {
            if !self.face_contains_point_strict(face, *point)? {
                return Err(TopologyError::PolylineLeavesFace);
            }
        }
        for index in 0..loop_points.len() {
            let start = loop_points[index];
            let end = loop_points[(index + 1) % loop_points.len()];
            let midpoint = interpolate(start, end, 0.5);
            if !self.face_contains_point_strict(face, midpoint)? {
                return Err(TopologyError::PolylineLeavesFace);
            }
            for boundary in std::iter::once(&boundaries.outer).chain(boundaries.holes.iter()) {
                for boundary_index in 0..boundary.len() {
                    if segments_intersect(
                        start,
                        end,
                        boundary[boundary_index],
                        boundary[(boundary_index + 1) % boundary.len()],
                    ) {
                        return Err(TopologyError::PolylineLeavesFace);
                    }
                }
            }
        }

        let existing_holes = self.face(face)?.holes.clone();
        let mut remaining_holes = Vec::new();
        let mut transferred_holes = Vec::new();
        for hole in existing_holes {
            let sample = self.vertex(self.edge(hole).origin).point;
            if point_in_polygon(sample, &loop_points) {
                transferred_holes.push(hole);
            } else {
                remaining_holes.push(hole);
            }
        }

        let new_face =
            FaceId(u32::try_from(self.faces.len()).map_err(|_| TopologyError::TooManyVertices)?);
        let edge_base = self.half_edges.len();
        let vertex_base = self.vertices.len();
        let count = loop_points.len();
        let mut island_edges = Vec::with_capacity(count);
        let mut hole_edges = Vec::with_capacity(count);
        for index in 0..count {
            island_edges.push(HalfEdgeId(
                u32::try_from(edge_base + index * 2)
                    .map_err(|_| TopologyError::TooManyVertices)?,
            ));
            hole_edges.push(HalfEdgeId(
                u32::try_from(edge_base + index * 2 + 1)
                    .map_err(|_| TopologyError::TooManyVertices)?,
            ));
        }
        let mut loop_vertices = Vec::with_capacity(count);
        for (index, point) in loop_points.iter().copied().enumerate() {
            let vertex = VertexId(
                u32::try_from(vertex_base + index).map_err(|_| TopologyError::TooManyVertices)?,
            );
            self.vertices.push(TopologyVertex {
                point,
                outgoing: Some(island_edges[index]),
            });
            loop_vertices.push(vertex);
        }

        for index in 0..count {
            self.half_edges.push(HalfEdge {
                origin: loop_vertices[index],
                twin: Some(hole_edges[index]),
                next: island_edges[(index + 1) % count],
                prev: island_edges[(index + count - 1) % count],
                face: new_face,
            });
            self.half_edges.push(HalfEdge {
                origin: loop_vertices[(index + 1) % count],
                twin: Some(island_edges[index]),
                next: hole_edges[(index + count - 1) % count],
                prev: hole_edges[(index + 1) % count],
                face,
            });
        }

        self.face_mut(face).holes = remaining_holes;
        self.face_mut(face).holes.push(hole_edges[0]);
        self.faces.push(Face {
            outer: island_edges[0],
            holes: transferred_holes.clone(),
        });
        for hole in transferred_holes {
            self.assign_face_cycle(hole, new_face)?;
        }
        self.validate()?;
        Ok(new_face)
    }

'''
topology = replace_section(
    topology,
    "    pub fn triangulate_face(&self, face: FaceId)",
    "    pub fn split_face_with_segment(",
    triangulate,
    "triangulation and closed loop insertion",
)

validate = r'''    pub fn validate(&self) -> Result<(), TopologyError> {
        for (vertex_index, vertex) in self.vertices.iter().enumerate() {
            if !vertex.point.is_finite() {
                return Err(TopologyError::InvalidTopology);
            }
            if let Some(outgoing) = vertex.outgoing {
                if self.edge(outgoing).origin != VertexId(vertex_index as u32) {
                    return Err(TopologyError::InvalidTopology);
                }
            }
        }

        for (edge_index, edge) in self.half_edges.iter().enumerate() {
            let edge_id = HalfEdgeId(edge_index as u32);
            if self.edge(edge.next).prev != edge_id || self.edge(edge.prev).next != edge_id {
                return Err(TopologyError::InvalidTopology);
            }
            if edge.origin.0 as usize >= self.vertices.len()
                || edge.face.0 as usize >= self.faces.len()
            {
                return Err(TopologyError::InvalidTopology);
            }
            if let Some(twin) = edge.twin {
                let twin_edge = self.edge(twin);
                if twin_edge.twin != Some(edge_id)
                    || twin_edge.origin != self.edge(edge.next).origin
                    || self.edge(twin_edge.next).origin != edge.origin
                {
                    return Err(TopologyError::InvalidTopology);
                }
            }
        }

        let mut seen = vec![false; self.half_edges.len()];
        for face_index in 0..self.faces.len() {
            let face = FaceId(face_index as u32);
            let boundaries = self.face_boundary_half_edges(face)?;
            if boundaries.is_empty() {
                return Err(TopologyError::InvalidTopology);
            }
            for cycle in &boundaries {
                if cycle.len() < 3 || cycle.iter().any(|edge| self.edge(*edge).face != face) {
                    return Err(TopologyError::InvalidTopology);
                }
                for edge in cycle {
                    if seen[edge.0 as usize] {
                        return Err(TopologyError::InvalidTopology);
                    }
                    seen[edge.0 as usize] = true;
                }
            }

            let loops = self.face_boundary_loops(face)?;
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
            }
            for first in 0..loops.holes.len() {
                for second in (first + 1)..loops.holes.len() {
                    let a = &loops.holes[first];
                    let b = &loops.holes[second];
                    if polygons_intersect(a, b)
                        || point_in_polygon(a[0], b)
                        || point_in_polygon(b[0], a)
                    {
                        return Err(TopologyError::InvalidTopology);
                    }
                }
            }
        }
        for (edge_index, edge) in self.half_edges.iter().enumerate() {
            if !seen[edge_index] || edge.face.0 as usize >= self.faces.len() {
                return Err(TopologyError::InvalidTopology);
            }
        }
        Ok(())
    }

'''
topology = replace_section(
    topology,
    "    pub fn validate(&self) -> Result<(), TopologyError>",
    "    fn split_face_with_segment_in_place(",
    validate,
    "topology validation",
)

split_segment = r'''    fn split_face_with_segment_in_place(
        &mut self,
        face: FaceId,
        start: Point2,
        end: Point2,
    ) -> Result<FaceId, TopologyError> {
        if !start.is_finite() || !end.is_finite() {
            return Err(TopologyError::NonFinitePoint);
        }
        if squared_distance(start, end) <= EPSILON * EPSILON {
            return Err(TopologyError::DegenerateSegment);
        }
        self.face(face)?;
        let existing_holes = self.face(face)?.holes.clone();

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

        if self.edge(start_out).next == end_out || self.edge(end_out).next == start_out {
            return Err(TopologyError::SegmentDoesNotSplitFace);
        }
        self.validate_chord(face, start_vertex, end_vertex, start, end)?;

        let start_prev = self.edge(start_out).prev;
        let end_prev = self.edge(end_out).prev;
        let side_start_to_end = self.path_side(start_out, end_vertex, start, end)?;
        let new_face =
            FaceId(u32::try_from(self.faces.len()).map_err(|_| TopologyError::TooManyVertices)?);

        let start_to_end = HalfEdgeId(
            u32::try_from(self.half_edges.len()).map_err(|_| TopologyError::TooManyVertices)?,
        );
        let end_to_start = HalfEdgeId(start_to_end.0 + 1);
        self.half_edges.push(HalfEdge {
            origin: start_vertex,
            twin: Some(end_to_start),
            next: end_out,
            prev: start_prev,
            face,
        });
        self.half_edges.push(HalfEdge {
            origin: end_vertex,
            twin: Some(start_to_end),
            next: start_out,
            prev: end_prev,
            face,
        });

        self.edge_mut(start_prev).next = start_to_end;
        self.edge_mut(end_out).prev = start_to_end;
        self.edge_mut(end_prev).next = end_to_start;
        self.edge_mut(start_out).prev = end_to_start;

        let (old_boundary, new_boundary) = if side_start_to_end > 0.0 {
            (start_out, end_out)
        } else {
            (end_out, start_out)
        };
        self.faces.push(Face {
            outer: new_boundary,
            holes: Vec::new(),
        });
        self.face_mut(face).outer = old_boundary;
        self.face_mut(face).holes.clear();
        self.assign_face_cycle(old_boundary, face)?;
        self.assign_face_cycle(new_boundary, new_face)?;
        self.redistribute_holes_after_outer_split(face, new_face, existing_holes)?;
        self.validate()?;
        Ok(new_face)
    }

'''
topology = replace_section(
    topology,
    "    fn split_face_with_segment_in_place(",
    "    fn validate_chord(",
    split_segment,
    "segment split with holes",
)

validate_chord = r'''    fn validate_chord(
        &self,
        face: FaceId,
        start_vertex: VertexId,
        end_vertex: VertexId,
        start: Point2,
        end: Point2,
    ) -> Result<(), TopologyError> {
        let midpoint = interpolate(start, end, 0.5);
        if !self.face_contains_point_strict(face, midpoint)? {
            return Err(TopologyError::SegmentLeavesFace);
        }

        for boundary in self.face_boundary_half_edges(face)? {
            for edge_id in boundary {
                let edge = self.edge(edge_id);
                let edge_start_vertex = edge.origin;
                let edge_end_vertex = self.edge(edge.next).origin;
                let edge_start = self.vertex(edge_start_vertex).point;
                let edge_end = self.vertex(edge_end_vertex).point;
                let incident = edge_start_vertex == start_vertex
                    || edge_end_vertex == start_vertex
                    || edge_start_vertex == end_vertex
                    || edge_end_vertex == end_vertex;
                if incident {
                    let (shared, other, chord_other) = if edge_start_vertex == start_vertex {
                        (start, edge_end, end)
                    } else if edge_end_vertex == start_vertex {
                        (start, edge_start, end)
                    } else if edge_start_vertex == end_vertex {
                        (end, edge_end, start)
                    } else {
                        (end, edge_start, start)
                    };
                    let edge_direction = Point2::new(other.x - shared.x, other.y - shared.y);
                    let chord_direction =
                        Point2::new(chord_other.x - shared.x, chord_other.y - shared.y);
                    let overlaps_chord = is_collinear(start, end, other)
                        && edge_direction.x * chord_direction.x
                            + edge_direction.y * chord_direction.y
                            > squared_length_tolerance(shared, chord_other);
                    if overlaps_chord {
                        return Err(TopologyError::SegmentLeavesFace);
                    }
                    continue;
                }
                if segments_intersect(start, end, edge_start, edge_end) {
                    return Err(TopologyError::SegmentLeavesFace);
                }
            }
        }
        Ok(())
    }

'''
topology = replace_section(
    topology,
    "    fn validate_chord(",
    "    fn path_side(",
    validate_chord,
    "hole-aware chord validation",
)

split_polyline = r'''    fn split_face_with_polyline_in_place(
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
        let existing_holes = self.face(face)?.holes.clone();

        let polygon = self.face_polygon(face)?;
        let start = points[0];
        let end = *points.last().expect("path length checked");
        if !point_on_polygon_boundary(start, &polygon) || !point_on_polygon_boundary(end, &polygon)
        {
            return Err(TopologyError::PointOffBoundary);
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
            for hole in &self.face_boundary_loops(face)?.holes {
                for index in 0..hole.len() {
                    if segments_intersect(
                        segment[0],
                        segment[1],
                        hole[index],
                        hole[(index + 1) % hole.len()],
                    ) {
                        return Err(TopologyError::PolylineLeavesFace);
                    }
                }
            }
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

        self.faces.push(Face {
            outer: end_out,
            holes: Vec::new(),
        });
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
        self.face_mut(face).outer = start_out;
        self.face_mut(face).holes.clear();
        self.assign_face_cycle(start_out, face)?;
        self.assign_face_cycle(end_out, new_face)?;
        self.redistribute_holes_after_outer_split(face, new_face, existing_holes)?;
        self.validate()?;
        Ok(new_face)
    }

'''
topology = replace_section(
    topology,
    "    fn split_face_with_polyline_in_place(",
    "    /// Decomposes one simple open polyline",
    split_polyline,
    "polyline split with holes",
)

face_containing = r'''    fn face_containing_point_strict(&self, point: Point2) -> Result<FaceId, TopologyError> {
        let mut found = None;
        for face_index in 0..self.faces.len() {
            let face = FaceId(face_index as u32);
            if self.face_contains_point_strict(face, point)? {
                if found.is_some() {
                    return Err(TopologyError::InvalidTopology);
                }
                found = Some(face);
            }
        }
        found.ok_or(TopologyError::PolylineLeavesTopology)
    }

'''
topology = replace_section(
    topology,
    "    fn face_containing_point_strict(",
    "    fn point_on_any_face_boundary(",
    face_containing,
    "face containment",
)

cycle_helpers = r'''    fn face_half_edges(&self, face: FaceId) -> Result<Vec<HalfEdgeId>, TopologyError> {
        self.cycle_half_edges(self.face(face)?.outer, face)
    }

    fn face_boundary_half_edges(
        &self,
        face: FaceId,
    ) -> Result<Vec<Vec<HalfEdgeId>>, TopologyError> {
        let face_data = self.face(face)?;
        let mut boundaries = Vec::with_capacity(face_data.holes.len() + 1);
        boundaries.push(self.cycle_half_edges(face_data.outer, face)?);
        for hole in &face_data.holes {
            boundaries.push(self.cycle_half_edges(*hole, face)?);
        }
        Ok(boundaries)
    }

    fn cycle_half_edges(
        &self,
        boundary: HalfEdgeId,
        face: FaceId,
    ) -> Result<Vec<HalfEdgeId>, TopologyError> {
        let mut edges = Vec::new();
        let mut current = boundary;
        for _ in 0..=self.half_edges.len() {
            if self.edge(current).face != face {
                return Err(TopologyError::InvalidTopology);
            }
            edges.push(current);
            current = self.edge(current).next;
            if current == boundary {
                return Ok(edges);
            }
        }
        Err(TopologyError::InvalidTopology)
    }

    fn cycle_polygon(
        &self,
        boundary: HalfEdgeId,
        face: FaceId,
    ) -> Result<Vec<Point2>, TopologyError> {
        Ok(self
            .cycle_half_edges(boundary, face)?
            .into_iter()
            .map(|edge| self.vertex(self.edge(edge).origin).point)
            .collect())
    }

    fn face_contains_point_strict(
        &self,
        face: FaceId,
        point: Point2,
    ) -> Result<bool, TopologyError> {
        let boundaries = self.face_boundary_loops(face)?;
        if point_on_polygon_boundary(point, &boundaries.outer)
            || !point_in_polygon(point, &boundaries.outer)
        {
            return Ok(false);
        }
        for hole in boundaries.holes {
            if point_on_polygon_boundary(point, &hole) || point_in_polygon(point, &hole) {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn redistribute_holes_after_outer_split(
        &mut self,
        face: FaceId,
        new_face: FaceId,
        existing_holes: Vec<HalfEdgeId>,
    ) -> Result<(), TopologyError> {
        let old_outer = self.face_polygon(face)?;
        let new_outer = self.face_polygon(new_face)?;
        let mut old_holes = Vec::new();
        let mut new_holes = Vec::new();
        for hole in existing_holes {
            let sample = self.vertex(self.edge(hole).origin).point;
            let in_old = point_in_polygon(sample, &old_outer);
            let in_new = point_in_polygon(sample, &new_outer);
            match (in_old, in_new) {
                (true, false) => old_holes.push(hole),
                (false, true) => {
                    self.assign_face_cycle(hole, new_face)?;
                    new_holes.push(hole);
                }
                _ => return Err(TopologyError::InvalidTopology),
            }
        }
        self.face_mut(face).holes = old_holes;
        self.face_mut(new_face).holes = new_holes;
        Ok(())
    }

'''
topology = replace_section(
    topology,
    "    fn face_half_edges(&self, face: FaceId)",
    "    fn vertex(&self, id: VertexId)",
    cycle_helpers,
    "boundary cycle helpers",
)

# Free helpers: preserve existing simple ear clipping for hole-free faces.
free_helpers = r'''fn triangulate_simple_polygon(
    face: FaceId,
    mut vertices: Vec<Point2>,
) -> Result<FaceTriangulation, TopologyError> {
    if vertices.len() < 3 {
        return Err(TopologyError::TriangulationFailed(face));
    }
    if signed_area(&vertices) < 0.0 {
        vertices.reverse();
    }

    let mut remaining: Vec<usize> = (0..vertices.len()).collect();
    let mut triangles = Vec::with_capacity(vertices.len().saturating_sub(2));
    let max_iterations = vertices.len() * vertices.len();
    let mut iterations = 0;

    while remaining.len() > 3 {
        let mut clipped = false;
        for cursor in 0..remaining.len() {
            let previous = remaining[(cursor + remaining.len() - 1) % remaining.len()];
            let current = remaining[cursor];
            let next = remaining[(cursor + 1) % remaining.len()];
            let a = vertices[previous];
            let b = vertices[current];
            let c = vertices[next];
            if orientation(a, b, c) <= orientation_tolerance(a, b, c) {
                continue;
            }

            let contains_other = remaining.iter().copied().any(|candidate| {
                candidate != previous
                    && candidate != current
                    && candidate != next
                    && point_in_or_on_triangle(vertices[candidate], a, b, c)
            });
            if contains_other {
                continue;
            }

            triangles.push([
                u32::try_from(previous).map_err(|_| TopologyError::TooManyVertices)?,
                u32::try_from(current).map_err(|_| TopologyError::TooManyVertices)?,
                u32::try_from(next).map_err(|_| TopologyError::TooManyVertices)?,
            ]);
            remaining.remove(cursor);
            clipped = true;
            break;
        }

        iterations += 1;
        if !clipped || iterations > max_iterations {
            return Err(TopologyError::TriangulationFailed(face));
        }
    }

    triangles.push([
        u32::try_from(remaining[0]).map_err(|_| TopologyError::TooManyVertices)?,
        u32::try_from(remaining[1]).map_err(|_| TopologyError::TooManyVertices)?,
        u32::try_from(remaining[2]).map_err(|_| TopologyError::TooManyVertices)?,
    ]);
    Ok(FaceTriangulation {
        vertices,
        triangles,
    })
}

fn normalize_closed_loop(points: &[Point2]) -> Result<Vec<Point2>, TopologyError> {
    let mut loop_points = points.to_vec();
    if loop_points.len() > 1
        && approximately_equal(loop_points[0], *loop_points.last().expect("non-empty path"))
    {
        loop_points.pop();
    }
    if loop_points.len() < 3 {
        return Err(TopologyError::TooFewVertices);
    }
    if loop_points.iter().any(|point| !point.is_finite()) {
        return Err(TopologyError::NonFinitePoint);
    }
    if (0..loop_points.len()).any(|index| {
        approximately_equal(
            loop_points[index],
            loop_points[(index + 1) % loop_points.len()],
        )
    }) {
        return Err(TopologyError::DuplicateAdjacentVertex);
    }
    if !is_simple_polygon(&loop_points) {
        return Err(TopologyError::PolylineSelfIntersecting);
    }
    let area = signed_area(&loop_points);
    if area.abs() <= polygon_area_tolerance(&loop_points) {
        return Err(TopologyError::DegeneratePolygon);
    }
    if area < 0.0 {
        loop_points.reverse();
    }
    Ok(loop_points)
}

fn polygons_intersect(a: &[Point2], b: &[Point2]) -> bool {
    for first in 0..a.len() {
        for second in 0..b.len() {
            if segments_intersect(
                a[first],
                a[(first + 1) % a.len()],
                b[second],
                b[(second + 1) % b.len()],
            ) {
                return true;
            }
        }
    }
    false
}

'''
marker = "fn signed_area(points: &[Point2]) -> f32 {"
if "fn triangulate_simple_polygon" not in topology:
    if marker not in topology:
        raise SystemExit("signed area marker not found")
    topology = topology.replace(marker, free_helpers + marker, 1)

# Tests for holes, multiple holes, nested transfer, and open-split compatibility.
if "fn closed_loop_creates_hole_and_inner_face" not in topology:
    marker = '''    #[test]
    fn traces_path_across_existing_faces() {'''
    tests = r'''    fn triangle_area(a: Point2, b: Point2, c: Point2) -> f32 {
        orientation(a, b, c).abs() * 0.5
    }

    #[test]
    fn closed_loop_creates_hole_and_inner_face() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-2.0, -2.0),
            Point2::new(2.0, -2.0),
            Point2::new(2.0, 2.0),
            Point2::new(-2.0, 2.0),
        ])
        .unwrap();
        let inner = topology
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

        assert_eq!(inner, FaceId(1));
        assert_eq!(topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(), 1);
        assert!(topology.face_boundary_loops(inner).unwrap().holes.is_empty());
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
    fn face_can_own_multiple_disjoint_holes() {
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

        assert_eq!(topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(), 2);
        assert!(topology.triangulate_face(FaceId(0)).is_ok());
        topology.validate().unwrap();
    }

    #[test]
    fn enclosing_cut_transfers_existing_hole_to_inner_face() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-3.0, -3.0),
            Point2::new(3.0, -3.0),
            Point2::new(3.0, 3.0),
            Point2::new(-3.0, 3.0),
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
        let annulus = topology
            .insert_closed_loop(
                FaceId(0),
                &[
                    Point2::new(-1.5, -1.5),
                    Point2::new(1.5, -1.5),
                    Point2::new(1.5, 1.5),
                    Point2::new(-1.5, 1.5),
                ],
            )
            .unwrap();

        assert_eq!(topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(), 1);
        assert_eq!(topology.face_boundary_loops(annulus).unwrap().holes.len(), 1);
        assert!(topology.triangulate_face(annulus).is_ok());
        topology.validate().unwrap();
    }

    #[test]
    fn outer_chord_split_preserves_existing_hole() {
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
                    Point2::new(-1.5, -0.4),
                    Point2::new(-0.7, -0.4),
                    Point2::new(-0.7, 0.4),
                    Point2::new(-1.5, 0.4),
                ],
            )
            .unwrap();
        let child = topology
            .split_face_with_segment(
                FaceId(0),
                Point2::new(0.5, -2.0),
                Point2::new(0.5, 2.0),
            )
            .unwrap();

        let hole_counts = [
            topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
            topology.face_boundary_loops(child).unwrap().holes.len(),
        ];
        assert_eq!(hole_counts.iter().sum::<usize>(), 1);
        topology.validate().unwrap();
    }

    #[test]
    fn traces_path_across_existing_faces() {'''
    if marker not in topology:
        raise SystemExit("topology test marker not found")
    topology = topology.replace(marker, tests, 1)

topology_path.write_text(topology)


# Domain -------------------------------------------------------------------------
lib_path = Path("crates/kirigami-core/src/lib.rs")
lib = lib_path.read_text()
lib = replace_once(
    lib,
    '''pub use topology::{
    FaceId, FaceTriangulation, HalfEdgeId, PlanarTopology, TopologyError, VertexId,
};''',
    '''pub use topology::{
    FaceBoundaryLoops, FaceId, FaceTriangulation, HalfEdgeId, PlanarTopology, TopologyError,
    VertexId,
};''',
    "re-export face boundary loops",
)

closed_method = r'''    /// Cuts a simple closed path fully inside one panel. The enclosed material becomes
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

'''
marker = "    fn split_panel_with_path(\n"
if "pub fn cut_closed_path" not in lib:
    if marker not in lib:
        raise SystemExit("split panel path marker not found")
    lib = lib.replace(marker, closed_method + marker, 1)

# Adapt seam reattachment to all boundary loops rather than outer polygons only.
lib = lib.replace(
    '''        let a_polygon = self.topology.face_polygon(original_face)?;
        let b_polygon = self.topology.face_polygon(new_face)?;''',
    '''        let a_boundaries = self.topology.face_boundary_loops(original_face)?;
        let b_boundaries = self.topology.face_boundary_loops(new_face)?;''',
    1,
)
lib = lib.replace(
    '''            &a_polygon,
            &b_polygon,''',
    '''            &a_boundaries,
            &b_boundaries,''',
    1,
)
lib = lib.replace(
    '''        a_polygon: &[Point2],
        b_polygon: &[Point2],''',
    '''        a_boundaries: &FaceBoundaryLoops,
        b_boundaries: &FaceBoundaryLoops,''',
    1,
)
lib = lib.replace(
    '''                a_polygon,
                b_polygon,''',
    '''                a_boundaries,
                b_boundaries,''',
    1,
)
lib = lib.replace(
    '''    a_polygon: &[Point2],
    b_polygon: &[Point2],''',
    '''    a_boundaries: &FaceBoundaryLoops,
    b_boundaries: &FaceBoundaryLoops,''',
    1,
)
lib = lib.replace(
    '''        let on_a = point_on_boundary(midpoint, a_polygon);
        let on_b = point_on_boundary(midpoint, b_polygon);''',
    '''        let on_a = point_on_boundary_loops(midpoint, a_boundaries);
        let on_b = point_on_boundary_loops(midpoint, b_boundaries);''',
    1,
)

reattach_method = r'''    fn reattach_seams_inside_closed_cut(
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

'''
marker = "    fn reattached_seams_after_split(\n"
if "fn reattach_seams_inside_closed_cut" not in lib:
    if marker not in lib:
        raise SystemExit("seam reattachment marker not found")
    lib = lib.replace(marker, reattach_method + marker, 1)

# Replace outer-only boundary helper with loop-aware helper and add closed-path helpers.
old_boundary_helper = '''fn point_on_boundary(point: Point2, polygon: &[Point2]) -> bool {
    (0..polygon.len())
        .any(|index| point_on_segment(point, polygon[index], polygon[(index + 1) % polygon.len()]))
}
'''
new_boundary_helper = r'''fn point_on_boundary_loops(point: Point2, boundaries: &FaceBoundaryLoops) -> bool {
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
'''
lib = replace_once(lib, old_boundary_helper, new_boundary_helper, "domain boundary helpers")

# Domain tests.
if "fn closed_cut_creates_detached_panel_and_hole" not in lib:
    marker = '''    #[test]
    fn bent_cut_creates_segmented_operation_and_disconnects() {'''
    tests = r'''    #[test]
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
        assert_eq!(model.operations().last().unwrap().path.first(), model.operations().last().unwrap().path.last());
        assert_eq!(
            model.seams().iter().filter(|seam| seam.operation == cut).count(),
            4
        );
        assert_eq!(model.topology().face_boundary_loops(FaceId(0)).unwrap().holes.len(), 1);
        let snapshot = model.render_snapshot(None).unwrap();
        assert_eq!(snapshot.panel_count, 2);
        assert_eq!(snapshot.component_count, 2);
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
        assert_eq!(model.topology().face_boundary_loops(FaceId(0)).unwrap().holes.len(), 2);
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
        assert!(model
            .seams()
            .iter()
            .filter(|seam| seam.operation == inner_cut)
            .all(|seam| seam.panel_a == annulus_panel || seam.panel_b == annulus_panel));
        assert_eq!(
            model
                .topology()
                .face_boundary_loops(model.panels().iter().find(|panel| panel.id == annulus_panel).unwrap().face)
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
            model.seams().iter().filter(|seam| seam.operation == closed).count(),
            4
        );
        assert!(model.render_snapshot(None).is_ok());
        model.topology().validate().unwrap();
    }

    #[test]
    fn bent_cut_creates_segmented_operation_and_disconnects() {'''
    if marker not in lib:
        raise SystemExit("domain test marker not found")
    lib = lib.replace(marker, tests, 1)

lib_path.write_text(lib)


# WASM/browser proof -------------------------------------------------------------
wasm_path = Path("crates/kirigami-wasm/src/lib.rs")
wasm = wasm_path.read_text()
wasm = replace_section(
    wasm,
    "#[wasm_bindgen]\npub fn demo_snapshot",
    "\nfn js_error(",
    r'''#[wasm_bindgen]
pub fn demo_snapshot(angle_degrees: f32, mode: &str) -> Result<String, JsValue> {
    let mut model = PaperModel::rectangle(2.4, 1.5).map_err(js_error)?;
    let fold = match mode {
        "crease" => {
            let operation = model
                .split_panel_with_segment(
                    PanelId(0),
                    Point2::new(0.0, -0.75),
                    Point2::new(0.0, 0.75),
                    OperationKind::Crease,
                )
                .map_err(js_error)?;
            Some(FoldRequest {
                operation,
                angle_radians: angle_degrees.to_radians(),
            })
        }
        "cut" => {
            model
                .split_panel_with_segment(
                    PanelId(0),
                    Point2::new(0.0, -0.75),
                    Point2::new(0.0, 0.75),
                    OperationKind::Cut,
                )
                .map_err(js_error)?;
            None
        }
        "hole" => {
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
        _ => return Err(JsValue::from_str("mode must be 'crease', 'cut', or 'hole'")),
    };
    let snapshot = model.render_snapshot(fold).map_err(js_error)?;
    serde_json::to_string(&snapshot).map_err(js_error)
}
''',
    "WASM closed cut demo",
)
wasm_path.write_text(wasm)

index_path = Path("web/index.html")
index = index_path.read_text()
index = replace_once(
    index,
    '''          <option value="cut">Cut</option>''',
    '''          <option value="cut">Cut</option>
          <option value="hole">Closed cut / hole</option>''',
    "hole demo option",
)
index = replace_once(
    index,
    '''      <p class="lede">Explore the difference between a crease that preserves connectivity and a cut that changes paper topology.</p>''',
    '''      <p class="lede">Explore creases, open cuts, and closed cuts that create detached inner paper and true face holes.</p>''',
    "demo description",
)
index_path.write_text(index)

app_path = Path("web/app.js")
app = app_path.read_text()
app = replace_once(
    app,
    '''  angle.disabled = currentMode === "cut";''',
    '''  angle.disabled = currentMode !== "crease";''',
    "disable fold angle for cuts",
)
app_path.write_text(app)

# Roadmap ------------------------------------------------------------------------
roadmap_path = Path("ROADMAP.md")
roadmap = roadmap_path.read_text()
roadmap = replace_once(
    roadmap,
    '''- Deterministic multi-face path tracing with inserted boundary intersections; one logical operation spans all generated seam segments.''',
    '''- Deterministic multi-face path tracing with inserted boundary intersections; one logical operation spans all generated seam segments.
- Closed cuts that create detached inner panels plus authoritative outer/inner face boundary loops.
- Multiple disjoint or nested hole loops with deterministic hole-aware rendering triangulation.''',
    "roadmap implemented holes",
)
roadmap = replace_once(
    roadmap,
    '''1. Add closed cut paths / holes and multiple boundary loops; then upgrade triangulation to constrained Delaunay when mesh quality or physical simulation requires it while keeping boundary constraints authoritative.
2. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
3. Add multi-crease fold state and constraint propagation so genuinely bent creases have an authoritative solver. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
4. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
5. Add layer ordering and flat-foldability validation.
6. Add SVG/FOLD import/export and deterministic pattern fixtures.
7. Add inverse-design/optimization experiments only after forward topology and validity are well tested.''',
    '''1. Extend open-path arrangement to boundary bridges involving inner loops when a cut should open an annulus instead of splitting it into two faces.
2. Integrate the collision foundation for BVH-backed paper self-intersection queries; keep collision detection advisory to `kirigami-core` validity until the boundary is proven.
3. Upgrade rendering/physics triangulation to constrained Delaunay when mesh quality requires it; the outer/inner loops remain authoritative regardless of triangulator.
4. Add multi-crease fold state and constraint propagation so genuinely bent creases have an authoritative solver. Use the physics engine only for material/mechanical behavior, never as the authority for crease meaning.
5. Add first-party 3D rendering through the shared `3d-lab` renderer seam while retaining the lightweight Canvas renderer as a deterministic fallback/acceptance surface.
6. Add layer ordering and flat-foldability validation.
7. Add SVG/FOLD import/export and deterministic pattern fixtures.
8. Add inverse-design/optimization experiments only after forward topology and validity are well tested.''',
    "roadmap next slices",
)
roadmap = replace_once(
    roadmap,
    '''The immediate algorithm work is closed-path/hole arrangement, constrained triangulation with multiple boundary loops, BVH/self-intersection, and then multi-crease constraint solving plus layer ordering. Multi-face open-path intersection insertion is now part of the deterministic topology foundation.''',
    '''Closed-path/hole arrangement and multiple boundary loops are now part of the deterministic topology foundation. Hole-aware rendering uses Earcut as a replaceable consumer of authoritative loops; the next algorithm work is boundary-bridge arrangement, BVH/self-intersection, then constrained triangulation and multi-crease constraint solving.''',
    "roadmap algorithms",
)
roadmap_path.write_text(roadmap)
