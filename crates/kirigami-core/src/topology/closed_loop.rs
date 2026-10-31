use super::geometry::{
    BoundaryIntersection, approximately_equal, interpolate, normalize_closed_loop,
    point_in_polygon, point_on_polygon_boundary, point_on_segment, polyline_is_simple,
    segment_boundary_intersection, segments_intersect, squared_distance,
};
use super::{
    EPSILON, Face, FaceId, HalfEdge, HalfEdgeId, PlanarTopology, TopologyError, TopologyVertex,
    VertexId,
};
use crate::Point2;

impl PlanarTopology {
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

    pub(super) fn cycle_has_same_face_twin(
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

    pub(super) fn face_contains_point_strict(
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

    pub(super) fn redistribute_holes_after_outer_split(
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
}
