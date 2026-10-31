use super::{EPSILON, FaceId, FaceTriangulation, TopologyError};
use crate::Point2;

const PATH_PARAMETER_TOLERANCE: f32 = f32::EPSILON * 64.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum BoundaryIntersection {
    None,
    Point { t: f32, point: Point2 },
    Overlap,
}

pub(super) fn segment_boundary_intersection(
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

pub(super) fn valid_face_loop(points: &[Point2]) -> bool {
    points.len() >= 3
        && !points
            .windows(2)
            .any(|pair| approximately_equal(pair[0], pair[1]))
        && is_simple_polygon(points)
        && signed_area(points).abs() > polygon_area_tolerance(points)
}

pub(super) fn polyline_is_simple(points: &[Point2]) -> bool {
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

pub(super) fn triangulate_simple_polygon(
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

pub(super) fn normalize_closed_loop(points: &[Point2]) -> Result<Vec<Point2>, TopologyError> {
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

pub(super) fn polygons_intersect(a: &[Point2], b: &[Point2]) -> bool {
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

pub(super) fn signed_area(points: &[Point2]) -> f32 {
    let mut twice_area = 0.0;
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        twice_area += current.x * next.y - next.x * current.y;
    }
    twice_area * 0.5
}

pub(super) fn polygon_area_tolerance(points: &[Point2]) -> f32 {
    let max_edge_squared = (0..points.len())
        .map(|index| squared_distance(points[index], points[(index + 1) % points.len()]))
        .fold(0.0_f32, f32::max);
    max_edge_squared * f32::EPSILON * points.len() as f32 * 16.0
}

pub(super) fn orientation(a: Point2, b: Point2, c: Point2) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

pub(super) fn orientation_tolerance(a: Point2, b: Point2, c: Point2) -> f32 {
    let scale_squared = squared_distance(a, b)
        .max(squared_distance(a, c))
        .max(squared_distance(b, c));
    scale_squared * f32::EPSILON * 16.0
}

pub(super) fn is_collinear(a: Point2, b: Point2, c: Point2) -> bool {
    orientation(a, b, c).abs() <= orientation_tolerance(a, b, c)
}

pub(super) fn squared_length_tolerance(a: Point2, b: Point2) -> f32 {
    squared_distance(a, b) * f32::EPSILON * 16.0
}

pub(super) fn approximately_equal(a: Point2, b: Point2) -> bool {
    squared_distance(a, b) <= EPSILON * EPSILON
}

pub(super) fn squared_distance(a: Point2, b: Point2) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

pub(super) fn point_on_segment(point: Point2, start: Point2, end: Point2) -> bool {
    if !is_collinear(start, end, point) {
        return false;
    }
    let dot = (point.x - start.x) * (end.x - start.x) + (point.y - start.y) * (end.y - start.y);
    let length_squared = squared_distance(start, end);
    let projection_tolerance = length_squared * f32::EPSILON * 16.0;
    dot >= -projection_tolerance && dot <= length_squared + projection_tolerance
}

pub(super) fn point_on_polygon_boundary(point: Point2, polygon: &[Point2]) -> bool {
    (0..polygon.len())
        .any(|index| point_on_segment(point, polygon[index], polygon[(index + 1) % polygon.len()]))
}

pub(super) fn point_in_polygon(point: Point2, polygon: &[Point2]) -> bool {
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

pub(super) fn segments_intersect(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
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

pub(super) fn is_simple_polygon(points: &[Point2]) -> bool {
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

pub(super) fn interpolate(start: Point2, end: Point2, t: f32) -> Point2 {
    Point2::new(
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t,
    )
}
