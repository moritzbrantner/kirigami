use crate::{EPSILON, Point2};

pub(super) fn path_is_straight(path: &[Point2]) -> bool {
    if path.len() < 2 {
        return false;
    }
    let start = path[0];
    let end = *path.last().expect("path length checked");
    let axis_x = end.x - start.x;
    let axis_y = end.y - start.y;
    let axis_length_squared = axis_x * axis_x + axis_y * axis_y;
    if axis_length_squared <= EPSILON * EPSILON {
        return false;
    }
    let axis_length = axis_length_squared.sqrt();
    path[1..path.len() - 1].iter().all(|point| {
        let cross = axis_x * (point.y - start.y) - axis_y * (point.x - start.x);
        cross.abs() <= EPSILON * axis_length
    })
}

pub(super) fn point_on_segment(point: Point2, start: Point2, end: Point2) -> bool {
    let ab = Point2::new(end.x - start.x, end.y - start.y);
    let ap = Point2::new(point.x - start.x, point.y - start.y);
    let cross = ab.x * ap.y - ab.y * ap.x;
    if cross.abs() > EPSILON {
        return false;
    }
    let dot = ap.x * ab.x + ap.y * ab.y;
    let length_squared = ab.x * ab.x + ab.y * ab.y;
    dot >= -EPSILON && dot <= length_squared + EPSILON
}

pub(super) fn interpolate(start: Point2, end: Point2, t: f32) -> Point2 {
    Point2::new(
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t,
    )
}

pub(super) fn squared_distance(a: Point2, b: Point2) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}
