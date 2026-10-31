use crate::geometry::{point_on_segment, squared_distance};
use crate::{EPSILON, OperationKind, PaperModel, Point2};
use serde::Serialize;

const FLAT_FOLD_ANGLE_TOLERANCE: f32 = 1.0e-4;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LocalFlatFoldabilityScope {
    /// Geometry-only necessary conditions at interior crease vertices. Passing this
    /// report does not prove global flat-foldability, mountain/valley consistency,
    /// or a valid layer ordering.
    InteriorVertexNecessaryConditions,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum LocalFlatFoldabilityIssue {
    OddCreaseDegree {
        point: Point2,
        crease_degree: usize,
    },
    KawasakiViolation {
        point: Point2,
        crease_degree: usize,
        alternating_angle_sum_radians: f32,
        deviation_radians: f32,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LocalFlatFoldabilityReport {
    pub scope: LocalFlatFoldabilityScope,
    /// Interior crease junctions with at least three unique incident rays.
    pub checked_vertices: usize,
    /// Junctions on an external or cut material boundary are intentionally excluded
    /// because the interior single-vertex theorem does not apply there.
    pub skipped_material_boundary_vertices: usize,
    pub issues: Vec<LocalFlatFoldabilityIssue>,
}

impl LocalFlatFoldabilityReport {
    #[must_use]
    pub fn passes_local_checks(&self) -> bool {
        self.issues.is_empty()
    }
}

impl PaperModel {
    /// Checks geometry-only necessary conditions for locally flat-foldable interior
    /// crease vertices. This deliberately does not infer mountain/valley assignments
    /// or global layer order.
    pub fn local_flat_foldability_report(&self) -> LocalFlatFoldabilityReport {
        let mut vertices = Vec::<CreaseVertex>::new();
        for seam in self
            .seams
            .iter()
            .filter(|seam| seam.kind == OperationKind::Crease)
        {
            add_crease_ray(&mut vertices, seam.start, seam.end);
            add_crease_ray(&mut vertices, seam.end, seam.start);
        }

        let external_boundary = self.topology.external_boundary_segments();
        let mut checked_vertices = 0;
        let mut skipped_material_boundary_vertices = 0;
        let mut issues = Vec::new();

        for mut vertex in vertices
            .into_iter()
            .filter(|vertex| vertex.ray_angles.len() >= 3)
        {
            if self.point_on_material_boundary(vertex.point, &external_boundary) {
                skipped_material_boundary_vertices += 1;
                continue;
            }

            checked_vertices += 1;
            vertex.ray_angles.sort_by(f32::total_cmp);
            let crease_degree = vertex.ray_angles.len();
            if crease_degree % 2 != 0 {
                issues.push(LocalFlatFoldabilityIssue::OddCreaseDegree {
                    point: vertex.point,
                    crease_degree,
                });
                continue;
            }

            let mut alternating_angle_sum = 0.0;
            for index in 0..crease_degree {
                let start = vertex.ray_angles[index];
                let end = if index + 1 < crease_degree {
                    vertex.ray_angles[index + 1]
                } else {
                    vertex.ray_angles[0] + std::f32::consts::PI * 2.0
                };
                if index % 2 == 0 {
                    alternating_angle_sum += end - start;
                }
            }

            let deviation = (alternating_angle_sum - std::f32::consts::PI).abs();
            if deviation > FLAT_FOLD_ANGLE_TOLERANCE {
                issues.push(LocalFlatFoldabilityIssue::KawasakiViolation {
                    point: vertex.point,
                    crease_degree,
                    alternating_angle_sum_radians: alternating_angle_sum,
                    deviation_radians: deviation,
                });
            }
        }

        LocalFlatFoldabilityReport {
            scope: LocalFlatFoldabilityScope::InteriorVertexNecessaryConditions,
            checked_vertices,
            skipped_material_boundary_vertices,
            issues,
        }
    }

    fn point_on_material_boundary(&self, point: Point2, external_boundary: &[[Point2; 2]]) -> bool {
        external_boundary
            .iter()
            .any(|segment| point_on_segment(point, segment[0], segment[1]))
            || self
                .operations
                .iter()
                .filter(|operation| operation.kind == OperationKind::Cut)
                .any(|operation| {
                    operation
                        .path
                        .windows(2)
                        .any(|segment| point_on_segment(point, segment[0], segment[1]))
                })
    }
}

#[derive(Debug)]
struct CreaseVertex {
    point: Point2,
    ray_angles: Vec<f32>,
}

fn add_crease_ray(vertices: &mut Vec<CreaseVertex>, point: Point2, other: Point2) {
    let dx = other.x - point.x;
    let dy = other.y - point.y;
    if dx * dx + dy * dy <= EPSILON * EPSILON {
        return;
    }

    let mut angle = dy.atan2(dx);
    if angle < 0.0 {
        angle += std::f32::consts::PI * 2.0;
    }

    let vertex_index = vertices
        .iter()
        .position(|vertex| squared_distance(vertex.point, point) <= EPSILON * EPSILON)
        .unwrap_or_else(|| {
            vertices.push(CreaseVertex {
                point,
                ray_angles: Vec::new(),
            });
            vertices.len() - 1
        });
    let rays = &mut vertices[vertex_index].ray_angles;
    if !rays
        .iter()
        .any(|existing| circular_angle_distance(*existing, angle) <= FLAT_FOLD_ANGLE_TOLERANCE)
    {
        rays.push(angle);
    }
}

fn circular_angle_distance(left: f32, right: f32) -> f32 {
    let full_turn = std::f32::consts::PI * 2.0;
    let difference = (left - right).abs();
    difference.min(full_turn - difference)
}
