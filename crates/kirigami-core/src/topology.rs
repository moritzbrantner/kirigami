use crate::Point2;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Face {
    boundary: HalfEdgeId,
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
        if area.abs() <= EPSILON {
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
                boundary: HalfEdgeId(0),
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

    pub fn face_polygon(&self, face: FaceId) -> Result<Vec<Point2>, TopologyError> {
        let edges = self.face_half_edges(face)?;
        Ok(edges
            .into_iter()
            .map(|edge| self.vertex(self.edge(edge).origin).point)
            .collect())
    }

    pub fn triangulate_face(&self, face: FaceId) -> Result<FaceTriangulation, TopologyError> {
        let mut vertices = self.face_polygon(face)?;
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
                if orientation(a, b, c) <= EPSILON {
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

        for face_index in 0..self.faces.len() {
            let face = FaceId(face_index as u32);
            let edges = self.face_half_edges(face)?;
            if edges.len() < 3 || edges.iter().any(|edge| self.edge(*edge).face != face) {
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
            boundary: new_boundary,
        });
        self.face_mut(face).boundary = old_boundary;
        self.assign_face_cycle(old_boundary, face)?;
        self.assign_face_cycle(new_boundary, new_face)?;
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
        let polygon = self.face_polygon(face)?;
        let midpoint = interpolate(start, end, 0.5);
        if point_on_polygon_boundary(midpoint, &polygon) || !point_in_polygon(midpoint, &polygon) {
            return Err(TopologyError::SegmentLeavesFace);
        }

        for edge_id in self.face_half_edges(face)? {
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
                let other = if edge_start_vertex == start_vertex || edge_start_vertex == end_vertex
                {
                    edge_end
                } else {
                    edge_start
                };
                if orientation(start, end, other).abs() <= EPSILON {
                    return Err(TopologyError::SegmentLeavesFace);
                }
                continue;
            }
            if segments_intersect(start, end, edge_start, edge_end) {
                return Err(TopologyError::SegmentLeavesFace);
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
            let side = orientation(start, end, self.vertex(vertex).point);
            if side.abs() > EPSILON {
                return Ok(side);
            }
            current = self.edge(current).next;
        }
        Err(TopologyError::InvalidTopology)
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
        let boundary = self.face(face)?.boundary;
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

fn signed_area(points: &[Point2]) -> f32 {
    let mut twice_area = 0.0;
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        twice_area += current.x * next.y - next.x * current.y;
    }
    twice_area * 0.5
}

fn orientation(a: Point2, b: Point2, c: Point2) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
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
    if orientation(start, end, point).abs() > EPSILON {
        return false;
    }
    let dot = (point.x - start.x) * (end.x - start.x) + (point.y - start.y) * (end.y - start.y);
    let length_squared = squared_distance(start, end);
    dot >= -EPSILON && dot <= length_squared + EPSILON
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

    if ((ab_c > EPSILON && ab_d < -EPSILON) || (ab_c < -EPSILON && ab_d > EPSILON))
        && ((cd_a > EPSILON && cd_b < -EPSILON) || (cd_a < -EPSILON && cd_b > EPSILON))
    {
        return true;
    }
    (ab_c.abs() <= EPSILON && point_on_segment(c, a, b))
        || (ab_d.abs() <= EPSILON && point_on_segment(d, a, b))
        || (cd_a.abs() <= EPSILON && point_on_segment(a, c, d))
        || (cd_b.abs() <= EPSILON && point_on_segment(b, c, d))
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
    ab >= -EPSILON && bc >= -EPSILON && ca >= -EPSILON
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
