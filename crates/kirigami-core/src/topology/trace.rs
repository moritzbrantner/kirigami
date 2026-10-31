use super::geometry::{
    BoundaryIntersection, approximately_equal, interpolate, point_on_segment, polyline_is_simple,
    segment_boundary_intersection, squared_distance,
};
use super::{EPSILON, FaceId, HalfEdgeId, PlanarTopology, TopologyError};
use crate::Point2;

impl PlanarTopology {
    /// Decomposes one simple open polyline into ordered fragments, one per traversed
    /// face. Existing face-boundary intersections are inserted deterministically.
    /// The topology is not mutated by this trace.
    pub(crate) fn trace_polyline_across_faces(
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
    pub(crate) fn face_containing_fragment(
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
}

const PATH_POSITION_TOLERANCE: f64 = 1.0e-7;

#[derive(Debug, Clone, Copy)]
struct PathCrossing {
    position: f64,
    point: Point2,
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
