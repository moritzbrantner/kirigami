use super::geometry::{
    BoundaryIntersection, approximately_equal, point_in_polygon, point_on_polygon_boundary,
    polygon_area_tolerance, polygons_intersect, segment_boundary_intersection, signed_area,
};
use super::{FaceId, HalfEdgeId, PlanarTopology, TopologyError, VertexId};
use crate::Point2;

impl PlanarTopology {
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
}
