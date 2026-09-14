use crate::Point2;
use earcut::Earcut;
use std::fmt;

const EPSILON: f32 = 1.0e-5;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VertexId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct HalfEdgeId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FaceId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq)]
struct TopologyVertex {
    point: Point2,
    outgoing: Option<HalfEdgeId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct HalfEdge {
    origin: VertexId,
    twin: Option<HalfEdgeId>,
    next: HalfEdgeId,
    prev: HalfEdgeId,
    face: FaceId,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Face {
    outer: HalfEdgeId,
    holes: Vec<HalfEdgeId>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PlanarTopology {
    vertices: Vec<TopologyVertex>,
    half_edges: Vec<HalfEdge>,
    faces: Vec<Face>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceTriangulation {
    pub vertices: Vec<Point2>,
    pub triangles: Vec<[u32; 3]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceBoundaryLoops {
    pub outer: Vec<Point2>,
    pub holes: Vec<Vec<Point2>>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TopologyError {
    TooFewVertices,
    NonFinitePoint,
    DuplicateAdjacentVertex,
    DegeneratePolygon,
    SelfIntersectingPolygon,
    UnknownFace(FaceId),
    PointOffBoundary,
    DegenerateSegment,
    TooFewPathPoints,
    PolylineSelfIntersecting,
    PolylineLeavesFace,
    PolylineLeavesTopology,
    PolylineOverlapsBoundary,
    PolylineCrossingAmbiguous,
    BoundaryBridgeRequiresDistinctComponents,
    SegmentDoesNotSplitFace,
    SegmentLeavesFace,
    InvalidTopology,
    TriangulationFailed(FaceId),
    TooManyVertices,
}

impl fmt::Display for TopologyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooFewVertices => formatter.write_str("a face requires at least three vertices"),
            Self::NonFinitePoint => formatter.write_str("topology points must be finite"),
            Self::DuplicateAdjacentVertex => {
                formatter.write_str("adjacent topology vertices must be distinct")
            }
            Self::DegeneratePolygon => formatter.write_str("polygon area must be non-zero"),
            Self::SelfIntersectingPolygon => {
                formatter.write_str("polygon boundary must not self-intersect")
            }
            Self::UnknownFace(face) => write!(formatter, "unknown face {}", face.0),
            Self::PointOffBoundary => formatter.write_str("point is not on the face boundary"),
            Self::DegenerateSegment => formatter.write_str("segment endpoints must be distinct"),
            Self::TooFewPathPoints => formatter.write_str("a path requires at least two points"),
            Self::PolylineSelfIntersecting => {
                formatter.write_str("polyline must not self-intersect")
            }
            Self::PolylineLeavesFace => formatter
                .write_str("polyline must stay inside the selected face except at endpoints"),
            Self::PolylineLeavesTopology => {
                formatter.write_str("polyline leaves the current planar subdivision")
            }
            Self::PolylineOverlapsBoundary => formatter
                .write_str("polyline must cross face boundaries instead of overlapping them"),
            Self::PolylineCrossingAmbiguous => formatter
                .write_str("polyline touches a face boundary without crossing into another face"),
            Self::BoundaryBridgeRequiresDistinctComponents => formatter
                .write_str("boundary bridge endpoints must lie on distinct boundary components"),
            Self::SegmentDoesNotSplitFace => {
                formatter.write_str("segment does not divide the face into two faces")
            }
            Self::SegmentLeavesFace => {
                formatter.write_str("segment leaves or crosses the face boundary")
            }
            Self::InvalidTopology => formatter.write_str("half-edge topology invariants failed"),
            Self::TriangulationFailed(face) => {
                write!(formatter, "could not triangulate face {}", face.0)
            }
            Self::TooManyVertices => formatter.write_str("topology exceeds u32 index capacity"),
        }
    }
}

impl std::error::Error for TopologyError {}

impl PlanarTopology {
    pub fn from_polygon(mut points: Vec<Point2>) -> Result<Self, TopologyError> {
        if points.len() > 1 && approximately_equal(points[0], *points.last().unwrap()) {
            points.pop();
        }
        if points.len() < 3 {
            return Err(TopologyError::TooFewVertices);
        }
        if points.iter().any(|point| !point.is_finite()) {
            return Err(TopologyError::NonFinitePoint);
        }
        if (0..points.len())
            .any(|index| approximately_equal(points[index], points[(index + 1) % points.len()]))
        {
            return Err(TopologyError::DuplicateAdjacentVertex);
        }
        if !is_simple_polygon(&points) {
            return Err(TopologyError::SelfIntersectingPolygon);
        }

        let area = signed_area(&points);
        if area.abs() <= polygon_area_tolerance(&points) {
            return Err(TopologyError::DegeneratePolygon);
        }
        if area < 0.0 {
            points.reverse();
        }

        let vertex_count =
            u32::try_from(points.len()).map_err(|_| TopologyError::TooManyVertices)?;
        let face = FaceId(0);
        let mut vertices = Vec::with_capacity(points.len());
        let mut half_edges = Vec::with_capacity(points.len());
        for (index, point) in points.into_iter().enumerate() {
            let edge = HalfEdgeId(index as u32);
            vertices.push(TopologyVertex {
                point,
                outgoing: Some(edge),
            });
        }
        for index in 0..vertex_count {
            half_edges.push(HalfEdge {
                origin: VertexId(index),
                twin: None,
                next: HalfEdgeId((index + 1) % vertex_count),
                prev: HalfEdgeId((index + vertex_count - 1) % vertex_count),
                face,
            });
        }

        let topology = Self {
            vertices,
            half_edges,
            faces: vec![Face {
                outer: HalfEdgeId(0),
                holes: Vec::new(),
            }],
        };
        topology.validate()?;
        Ok(topology)
    }

    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn half_edge_count(&self) -> usize {
        self.half_edges.len()
    }

    pub fn boundary_component_count(&self, face: FaceId) -> Result<usize, TopologyError> {
        let face = self.face(face)?;
        Ok(face.holes.len() + 1)
    }

    pub fn face_polygon(&self, face: FaceId) -> Result<Vec<Point2>, TopologyError> {
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

    pub fn triangulate_face(&self, face: FaceId) -> Result<FaceTriangulation, TopologyError> {
        let face_data = self.face(face)?;
        let mut has_bridge = self.cycle_has_same_face_twin(face_data.outer, face)?;
        for hole in &face_data.holes {
            has_bridge |= self.cycle_has_same_face_twin(*hole, face)?;
        }
        let boundaries = self.face_boundary_loops(face)?;
        if boundaries.holes.is_empty() && !has_bridge {
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
            hole_indices
                .push(u32::try_from(vertices.len()).map_err(|_| TopologyError::TooManyVertices)?);
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
                u32::try_from(edge_base + index * 2).map_err(|_| TopologyError::TooManyVertices)?,
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

        if points[1..points.len() - 1].iter().any(|point| {
            !self
                .face_contains_point_strict(face, *point)
                .unwrap_or(false)
        }) {
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
                        segment[0], segment[1], edge_start, edge_end,
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
            let removed_hole = if start_root == outer {
                end_root
            } else {
                start_root
            };
            self.face_mut(face)
                .holes
                .retain(|root| *root != removed_hole);
        } else {
            self.face_mut(face).holes.retain(|root| *root != end_root);
        }

        self.validate()?;
        Ok(())
    }

    pub fn split_face_with_segment(
        &mut self,
        face: FaceId,
        start: Point2,
        end: Point2,
    ) -> Result<FaceId, TopologyError> {
        let mut candidate = self.clone();
        let new_face = candidate.split_face_with_segment_in_place(face, start, end)?;
        *self = candidate;
        Ok(new_face)
    }

    pub fn validate(&self) -> Result<(), TopologyError> {
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

            let face_data = self.face(face)?;
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

    fn split_face_with_segment_in_place(
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

    fn validate_chord(
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

    fn path_side(
        &self,
        start_edge: HalfEdgeId,
        end_vertex: VertexId,
        start: Point2,
        end: Point2,
    ) -> Result<f32, TopologyError> {
        let mut current = self.edge(start_edge).next;
        for _ in 0..self.half_edges.len() {
            let vertex = self.edge(current).origin;
            if vertex == end_vertex {
                return Err(TopologyError::SegmentDoesNotSplitFace);
            }
            let point = self.vertex(vertex).point;
            let side = orientation(start, end, point);
            if side.abs() > orientation_tolerance(start, end, point) {
                return Ok(side);
            }
            current = self.edge(current).next;
        }
        Err(TopologyError::InvalidTopology)
    }

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
        Ok(self
            .cycle_half_edges(boundary, face)?
            .into_iter()
            .any(|edge_id| {
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

    fn locate_or_insert_boundary_vertex(
        &mut self,
        face: FaceId,
        point: Point2,
    ) -> Result<VertexId, TopologyError> {
        let boundary = self.face_half_edges(face)?;
        for edge_id in &boundary {
            let origin = self.edge(*edge_id).origin;
            if approximately_equal(self.vertex(origin).point, point) {
                return Ok(origin);
            }
        }
        for edge_id in boundary {
            let edge = self.edge(edge_id);
            let start = self.vertex(edge.origin).point;
            let end = self.vertex(self.edge(edge.next).origin).point;
            if point_on_segment(point, start, end) {
                return self.split_half_edge(edge_id, point);
            }
        }
        Err(TopologyError::PointOffBoundary)
    }

    fn split_half_edge(
        &mut self,
        edge_id: HalfEdgeId,
        point: Point2,
    ) -> Result<VertexId, TopologyError> {
        let edge = *self.edge(edge_id);
        let destination = self.edge(edge.next).origin;
        if approximately_equal(point, self.vertex(edge.origin).point) {
            return Ok(edge.origin);
        }
        if approximately_equal(point, self.vertex(destination).point) {
            return Ok(destination);
        }

        let vertex = VertexId(
            u32::try_from(self.vertices.len()).map_err(|_| TopologyError::TooManyVertices)?,
        );
        let forward = HalfEdgeId(
            u32::try_from(self.half_edges.len()).map_err(|_| TopologyError::TooManyVertices)?,
        );
        self.vertices.push(TopologyVertex {
            point,
            outgoing: Some(forward),
        });

        match edge.twin {
            None => {
                self.half_edges.push(HalfEdge {
                    origin: vertex,
                    twin: None,
                    next: edge.next,
                    prev: edge_id,
                    face: edge.face,
                });
                self.edge_mut(edge_id).next = forward;
                self.edge_mut(edge.next).prev = forward;
            }
            Some(twin_id) => {
                let twin = *self.edge(twin_id);
                let reverse = HalfEdgeId(forward.0 + 1);
                self.half_edges.push(HalfEdge {
                    origin: vertex,
                    twin: Some(twin_id),
                    next: edge.next,
                    prev: edge_id,
                    face: edge.face,
                });
                self.half_edges.push(HalfEdge {
                    origin: vertex,
                    twin: Some(edge_id),
                    next: twin.next,
                    prev: twin_id,
                    face: twin.face,
                });

                self.edge_mut(edge_id).next = forward;
                self.edge_mut(edge_id).twin = Some(reverse);
                self.edge_mut(edge.next).prev = forward;

                self.edge_mut(twin_id).next = reverse;
                self.edge_mut(twin_id).twin = Some(forward);
                self.edge_mut(twin.next).prev = reverse;
            }
        }
        self.validate()?;
        Ok(vertex)
    }

    fn assign_face_cycle(
        &mut self,
        boundary: HalfEdgeId,
        face: FaceId,
    ) -> Result<(), TopologyError> {
        let mut current = boundary;
        for _ in 0..=self.half_edges.len() {
            self.edge_mut(current).face = face;
            current = self.edge(current).next;
            if current == boundary {
                return Ok(());
            }
        }
        Err(TopologyError::InvalidTopology)
    }

    fn face_half_edges(&self, face: FaceId) -> Result<Vec<HalfEdgeId>, TopologyError> {
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

    fn vertex(&self, id: VertexId) -> &TopologyVertex {
        &self.vertices[id.0 as usize]
    }

    fn edge(&self, id: HalfEdgeId) -> &HalfEdge {
        &self.half_edges[id.0 as usize]
    }

    fn edge_mut(&mut self, id: HalfEdgeId) -> &mut HalfEdge {
        &mut self.half_edges[id.0 as usize]
    }

    fn face(&self, id: FaceId) -> Result<&Face, TopologyError> {
        self.faces
            .get(id.0 as usize)
            .ok_or(TopologyError::UnknownFace(id))
    }

    fn face_mut(&mut self, id: FaceId) -> &mut Face {
        &mut self.faces[id.0 as usize]
    }
}

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
        let existing_holes = self.face(face)?.holes.clone();

        let polygon = self.face_polygon(face)?;
        let start = points[0];
        let end = *points.last().expect("path length checked");
        if !point_on_polygon_boundary(start, &polygon) || !point_on_polygon_boundary(end, &polygon)
        {
            return Err(TopologyError::PointOffBoundary);
        }
        if points[1..points.len() - 1].iter().any(|point| {
            !self
                .face_contains_point_strict(face, *point)
                .unwrap_or(false)
        }) {
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
                match segment_boundary_intersection(segment[0], segment[1], edge_start, edge_end) {
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
            if self.face_contains_point_strict(face, point)? {
                if found.is_some() {
                    return Err(TopologyError::InvalidTopology);
                }
                found = Some(face);
            }
        }
        found.ok_or(TopologyError::PolylineLeavesTopology)
    }

    fn point_on_any_face_boundary(&self, point: Point2) -> bool {
        self.half_edges
            .iter()
            .enumerate()
            .any(|(edge_index, edge)| {
                let edge_id = HalfEdgeId(edge_index as u32);
                if edge.twin.is_some_and(|twin| twin.0 < edge_id.0) {
                    return false;
                }
                let start = self.vertex(edge.origin).point;
                let end = self.vertex(self.edge(edge.next).origin).point;
                point_on_segment(point, start, end)
            })
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
            && !approximately_equal(*fragment.last().expect("fragment has start"), point)
        {
            fragment.push(point);
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
        let t0 = ((edge_start.x - start.x) * rx + (edge_start.y - start.y) * ry) / length_squared;
        let t1 = ((edge_end.x - start.x) * rx + (edge_end.y - start.y) * ry) / length_squared;
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

fn triangulate_simple_polygon(
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

fn signed_area(points: &[Point2]) -> f32 {
    let mut twice_area = 0.0;
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        twice_area += current.x * next.y - next.x * current.y;
    }
    twice_area * 0.5
}

fn polygon_area_tolerance(points: &[Point2]) -> f32 {
    let max_edge_squared = (0..points.len())
        .map(|index| squared_distance(points[index], points[(index + 1) % points.len()]))
        .fold(0.0_f32, f32::max);
    max_edge_squared * f32::EPSILON * points.len() as f32 * 16.0
}

fn orientation(a: Point2, b: Point2, c: Point2) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn orientation_tolerance(a: Point2, b: Point2, c: Point2) -> f32 {
    let scale_squared = squared_distance(a, b)
        .max(squared_distance(a, c))
        .max(squared_distance(b, c));
    scale_squared * f32::EPSILON * 16.0
}

fn is_collinear(a: Point2, b: Point2, c: Point2) -> bool {
    orientation(a, b, c).abs() <= orientation_tolerance(a, b, c)
}

fn squared_length_tolerance(a: Point2, b: Point2) -> f32 {
    squared_distance(a, b) * f32::EPSILON * 16.0
}

fn approximately_equal(a: Point2, b: Point2) -> bool {
    squared_distance(a, b) <= EPSILON * EPSILON
}

fn squared_distance(a: Point2, b: Point2) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

fn point_on_segment(point: Point2, start: Point2, end: Point2) -> bool {
    if !is_collinear(start, end, point) {
        return false;
    }
    let dot = (point.x - start.x) * (end.x - start.x) + (point.y - start.y) * (end.y - start.y);
    let length_squared = squared_distance(start, end);
    let projection_tolerance = length_squared * f32::EPSILON * 16.0;
    dot >= -projection_tolerance && dot <= length_squared + projection_tolerance
}

fn point_on_polygon_boundary(point: Point2, polygon: &[Point2]) -> bool {
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

fn segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    let ab_c = orientation(a, b, c);
    let ab_d = orientation(a, b, d);
    let cd_a = orientation(c, d, a);
    let cd_b = orientation(c, d, b);
    let ab_c_tolerance = orientation_tolerance(a, b, c);
    let ab_d_tolerance = orientation_tolerance(a, b, d);
    let cd_a_tolerance = orientation_tolerance(c, d, a);
    let cd_b_tolerance = orientation_tolerance(c, d, b);

    if ((ab_c > ab_c_tolerance && ab_d < -ab_d_tolerance)
        || (ab_c < -ab_c_tolerance && ab_d > ab_d_tolerance))
        && ((cd_a > cd_a_tolerance && cd_b < -cd_b_tolerance)
            || (cd_a < -cd_a_tolerance && cd_b > cd_b_tolerance))
    {
        return true;
    }
    (ab_c.abs() <= ab_c_tolerance && point_on_segment(c, a, b))
        || (ab_d.abs() <= ab_d_tolerance && point_on_segment(d, a, b))
        || (cd_a.abs() <= cd_a_tolerance && point_on_segment(a, c, d))
        || (cd_b.abs() <= cd_b_tolerance && point_on_segment(b, c, d))
}

fn is_simple_polygon(points: &[Point2]) -> bool {
    for first in 0..points.len() {
        let first_next = (first + 1) % points.len();
        for second in (first + 1)..points.len() {
            let second_next = (second + 1) % points.len();
            if first == second
                || first_next == second
                || second_next == first
                || (first == 0 && second_next == 0)
            {
                continue;
            }
            if segments_intersect(
                points[first],
                points[first_next],
                points[second],
                points[second_next],
            ) {
                return false;
            }
        }
    }
    true
}

fn point_in_or_on_triangle(point: Point2, a: Point2, b: Point2, c: Point2) -> bool {
    let ab = orientation(a, b, point);
    let bc = orientation(b, c, point);
    let ca = orientation(c, a, point);
    ab >= -orientation_tolerance(a, b, point)
        && bc >= -orientation_tolerance(b, c, point)
        && ca >= -orientation_tolerance(c, a, point)
}

fn interpolate(start: Point2, end: Point2, t: f32) -> Point2 {
    Point2::new(
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn half_edge_split_preserves_invariants_and_shared_twins() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-1.0, -0.5),
            Point2::new(1.0, -0.5),
            Point2::new(1.0, 0.5),
            Point2::new(-1.0, 0.5),
        ])
        .unwrap();
        let second = topology
            .split_face_with_segment(FaceId(0), Point2::new(0.0, -0.5), Point2::new(0.0, 0.5))
            .unwrap();
        topology
            .split_face_with_segment(FaceId(0), Point2::new(-1.0, 0.0), Point2::new(0.0, 0.0))
            .unwrap();

        topology.validate().unwrap();
        assert_eq!(second, FaceId(1));
        assert_eq!(topology.face_count(), 3);
        assert_eq!(topology.vertex_count(), 8);
        assert!(topology.half_edges.iter().any(|edge| edge.twin.is_some()));
    }

    #[test]
    fn ear_clipping_triangulates_concave_face_without_crossing_boundary() {
        let topology = PlanarTopology::from_polygon(vec![
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(2.0, 1.0),
            Point2::new(1.0, 0.4),
            Point2::new(0.0, 1.0),
        ])
        .unwrap();
        let triangulation = topology.triangulate_face(FaceId(0)).unwrap();
        assert_eq!(triangulation.triangles.len(), 3);
    }

    #[test]
    fn accepts_small_non_degenerate_face() {
        let topology = PlanarTopology::from_polygon(vec![
            Point2::new(0.0, 0.0),
            Point2::new(0.001, 0.0),
            Point2::new(0.001, 0.001),
            Point2::new(0.0, 0.001),
        ])
        .unwrap();
        assert_eq!(topology.face_count(), 1);
        assert_eq!(
            topology
                .triangulate_face(FaceId(0))
                .unwrap()
                .triangles
                .len(),
            2
        );
        topology.validate().unwrap();
    }

    #[test]
    fn allows_chord_continuing_past_reflex_edge() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(2.0, 2.0),
            Point2::new(1.0, 2.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ])
        .unwrap();
        topology
            .split_face_with_segment(FaceId(0), Point2::new(1.0, 1.0), Point2::new(1.0, 0.0))
            .unwrap();
        assert_eq!(topology.face_count(), 2);
        topology.validate().unwrap();
    }

    fn triangle_area(a: Point2, b: Point2, c: Point2) -> f32 {
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
        assert_eq!(
            topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
            1
        );
        assert!(
            topology
                .face_boundary_loops(inner)
                .unwrap()
                .holes
                .is_empty()
        );
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

        assert_eq!(
            topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
            2
        );
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

        assert_eq!(
            topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
            1
        );
        assert_eq!(
            topology.face_boundary_loops(annulus).unwrap().holes.len(),
            1
        );
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
            .split_face_with_segment(FaceId(0), Point2::new(0.5, -2.0), Point2::new(0.5, 2.0))
            .unwrap();

        let hole_counts = [
            topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
            topology.face_boundary_loops(child).unwrap().holes.len(),
        ];
        assert_eq!(hole_counts.iter().sum::<usize>(), 1);
        topology.validate().unwrap();
    }

    #[test]
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
        assert!(
            topology
                .face_boundary_loops(FaceId(0))
                .unwrap()
                .holes
                .is_empty()
        );
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
        assert_eq!(
            topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
            1
        );
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
    fn traces_path_across_existing_faces() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(-1.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 1.0),
            Point2::new(-1.0, 1.0),
        ])
        .unwrap();
        topology
            .split_face_with_segment(FaceId(0), Point2::new(0.0, -1.0), Point2::new(0.0, 1.0))
            .unwrap();

        let fragments = topology
            .trace_polyline_across_faces(&[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)])
            .unwrap();
        assert_eq!(fragments.len(), 2);
        assert!(approximately_equal(
            *fragments[0].last().unwrap(),
            Point2::new(0.0, 0.0)
        ));
        assert!(approximately_equal(fragments[1][0], Point2::new(0.0, 0.0)));
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
            .split_face_with_segment(FaceId(0), Point2::new(0.0, -1.0), Point2::new(0.0, 1.0))
            .unwrap();
        let before = topology.clone();

        assert_eq!(
            topology
                .trace_polyline_across_faces(&[Point2::new(0.0, -0.75), Point2::new(0.0, 0.75),]),
            Err(TopologyError::PolylineOverlapsBoundary)
        );
        assert_eq!(topology, before);
    }

    #[test]
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
    fn rejects_chord_that_leaves_concave_face_without_mutation() {
        let mut topology = PlanarTopology::from_polygon(vec![
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(2.0, 2.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 2.0),
        ])
        .unwrap();
        let before = topology.clone();
        assert_eq!(
            topology.split_face_with_segment(
                FaceId(0),
                Point2::new(0.0, 1.8),
                Point2::new(2.0, 1.8),
            ),
            Err(TopologyError::SegmentLeavesFace)
        );
        assert_eq!(topology, before);
    }
}
