use super::geometry::{
    approximately_equal, interpolate, is_collinear, orientation, orientation_tolerance,
    point_on_polygon_boundary, point_on_segment, polyline_is_simple, segments_intersect,
    squared_distance, squared_length_tolerance, valid_face_loop,
};
use super::{
    EPSILON, Face, FaceId, HalfEdge, HalfEdgeId, PlanarTopology, TopologyError, TopologyVertex,
    VertexId,
};
use crate::Point2;

impl PlanarTopology {
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

    pub(super) fn split_half_edge(
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
