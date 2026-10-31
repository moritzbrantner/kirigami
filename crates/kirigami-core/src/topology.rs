mod closed_loop;
mod geometry;
mod split;
#[cfg(test)]
mod tests;
mod trace;
mod validate;

use crate::Point2;
use earcut::Earcut;
use geometry::{
    approximately_equal, is_simple_polygon, polygon_area_tolerance, signed_area,
    triangulate_simple_polygon,
};
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

/// One undirected topology edge expressed with authoritative vertex identities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TopologyEdge {
    pub start: VertexId,
    pub end: VertexId,
    /// Twin-less edges are the external material boundary.
    pub external_boundary: bool,
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

    /// Returns the geometric material boundary as deterministic directed segments.
    ///
    /// Internal cut/crease subdivision edges always have twins. Twin-less half-edges
    /// are therefore the original external sheet boundary even after later operations
    /// subdivide it.
    pub fn external_boundary_segments(&self) -> Vec<[Point2; 2]> {
        self.half_edges
            .iter()
            .filter(|edge| edge.twin.is_none())
            .map(|edge| {
                let start = self.vertex(edge.origin).point;
                let end = self.vertex(self.edge(edge.next).origin).point;
                [start, end]
            })
            .collect()
    }

    pub fn vertex_point(&self, id: VertexId) -> Option<Point2> {
        self.vertices.get(id.0 as usize).map(|vertex| vertex.point)
    }

    /// Returns every undirected edge once, in deterministic half-edge order.
    ///
    /// Operations that end on an existing edge split it, so these edges already carry
    /// every junction vertex and share vertex identities with the rest of the graph.
    pub fn undirected_edges(&self) -> Vec<TopologyEdge> {
        self.half_edges
            .iter()
            .enumerate()
            .filter(|(index, edge)| edge.twin.is_none_or(|twin| twin.0 as usize > *index))
            .map(|(_, edge)| TopologyEdge {
                start: edge.origin,
                end: self.edge(edge.next).origin,
                external_boundary: edge.twin.is_none(),
            })
            .collect()
    }

    pub fn boundary_component_count(&self, face: FaceId) -> Result<usize, TopologyError> {
        let face = self.face(face)?;
        Ok(face.holes.len() + 1)
    }

    /// Returns whether two faces intentionally meet at at least one authoritative
    /// topology vertex, including vertices on hole or bridge boundary walks.
    pub fn faces_share_vertex(&self, left: FaceId, right: FaceId) -> Result<bool, TopologyError> {
        let left_vertices = self.face_vertex_ids(left)?;
        let right_vertices = self.face_vertex_ids(right)?;
        Ok(left_vertices
            .iter()
            .any(|vertex| right_vertices.contains(vertex)))
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

    fn face_vertex_ids(&self, face: FaceId) -> Result<Vec<VertexId>, TopologyError> {
        let mut vertices = Vec::new();
        for boundary in self.face_boundary_half_edges(face)? {
            for edge_id in boundary {
                let vertex = self.edge(edge_id).origin;
                if !vertices.contains(&vertex) {
                    vertices.push(vertex);
                }
            }
        }
        Ok(vertices)
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
