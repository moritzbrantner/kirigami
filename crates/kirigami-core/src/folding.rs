use crate::geometry::path_is_straight;
use crate::{
    EPSILON, FoldRequest, ModelError, OperationId, OperationKind, PanelId, PaperModel,
    visit_connected,
};
use std::collections::HashSet;
use three_d_core::Vec3;

impl PaperModel {
    pub(super) fn resolve_folds(
        &self,
        requests: &[FoldRequest],
    ) -> Result<Vec<ResolvedFold>, ModelError> {
        let mut ordered = requests.to_vec();
        ordered.sort_by_key(|request| request.operation.0);
        if let Some(duplicate) = ordered
            .windows(2)
            .find(|pair| pair[0].operation == pair[1].operation)
        {
            return Err(ModelError::DuplicateFoldOperation(duplicate[0].operation));
        }

        let mut resolved = Vec::with_capacity(ordered.len());
        for request in ordered {
            if !request.angle_radians.is_finite() {
                return Err(ModelError::NonFiniteFoldAngle);
            }
            let operation = self
                .operations
                .iter()
                .find(|operation| operation.id == request.operation)
                .ok_or(ModelError::UnknownOperation(request.operation))?;
            if operation.kind == OperationKind::Cut {
                return Err(ModelError::CannotFoldCut(request.operation));
            }
            if !path_is_straight(&operation.path) {
                return Err(ModelError::BentCreaseRequiresConstraintSolver);
            }

            let moving_seeds: HashSet<PanelId> = self
                .seams
                .iter()
                .filter(|seam| seam.operation == operation.id)
                .map(|seam| seam.panel_b)
                .collect();
            let moving_panels =
                self.connected_panels_without_operation(&moving_seeds, operation.id);
            let (axis_start, axis_end) =
                self.transformed_operation_axis(operation.id, &resolved)?;

            resolved.push(ResolvedFold {
                axis_start,
                axis_end,
                angle_radians: request.angle_radians,
                moving_panels,
            });
        }
        Ok(resolved)
    }

    fn transformed_operation_axis(
        &self,
        operation: OperationId,
        resolved: &[ResolvedFold],
    ) -> Result<(Vec3, Vec3), ModelError> {
        let mut axis = None;
        let mut transformed_points = Vec::new();

        for seam in self.seams.iter().filter(|seam| seam.operation == operation) {
            let flat_start = Vec3::new(seam.start.x, seam.start.y, 0.0);
            let flat_end = Vec3::new(seam.end.x, seam.end.y, 0.0);
            let start_a = Self::transform_point_for_panel(resolved, seam.panel_a, flat_start);
            let end_a = Self::transform_point_for_panel(resolved, seam.panel_a, flat_end);
            let start_b = Self::transform_point_for_panel(resolved, seam.panel_b, flat_start);
            let end_b = Self::transform_point_for_panel(resolved, seam.panel_b, flat_end);

            if !points_near_3d(start_a, start_b) || !points_near_3d(end_a, end_b) {
                return Err(ModelError::FoldConstraintConflict(operation));
            }

            axis.get_or_insert((start_a, end_a));
            transformed_points.push(start_a);
            transformed_points.push(end_a);
        }

        let (axis_start, axis_end) = axis.ok_or(ModelError::FoldConstraintConflict(operation))?;
        if transformed_points
            .into_iter()
            .any(|point| !point_on_axis_3d(point, axis_start, axis_end))
        {
            return Err(ModelError::FoldConstraintConflict(operation));
        }
        Ok((axis_start, axis_end))
    }

    pub(super) fn transform_point_for_panel(
        resolved: &[ResolvedFold],
        panel: PanelId,
        mut point: Vec3,
    ) -> Vec3 {
        for fold in resolved {
            if fold.moving_panels.contains(&panel) {
                point = fold.transform(point);
            }
        }
        point
    }

    fn connected_panels_without_operation(
        &self,
        seeds: &HashSet<PanelId>,
        excluded: OperationId,
    ) -> HashSet<PanelId> {
        let adjacency = self.crease_adjacency(Some(excluded));
        let mut visited = seeds.clone();
        visit_connected(&adjacency, seeds.iter().copied().collect(), &mut visited);
        visited
    }
}

pub(super) struct ResolvedFold {
    axis_start: Vec3,
    axis_end: Vec3,
    angle_radians: f32,
    pub(super) moving_panels: HashSet<PanelId>,
}

impl ResolvedFold {
    fn transform(&self, point: Vec3) -> Vec3 {
        rotate_around_axis(point, self.axis_start, self.axis_end, self.angle_radians)
    }
}

fn rotate_around_axis(point: Vec3, axis_start: Vec3, axis_end: Vec3, angle: f32) -> Vec3 {
    let Some(axis) = (axis_end - axis_start).normalized() else {
        return point;
    };
    let relative = point - axis_start;
    let cosine = angle.cos();
    let sine = angle.sin();
    axis_start
        + relative * cosine
        + axis.cross(relative) * sine
        + axis * (axis.dot(relative) * (1.0 - cosine))
}

fn points_near_3d(left: Vec3, right: Vec3) -> bool {
    let delta = left - right;
    delta.dot(delta) <= EPSILON * EPSILON * 16.0
}

fn point_on_axis_3d(point: Vec3, axis_start: Vec3, axis_end: Vec3) -> bool {
    let Some(axis) = (axis_end - axis_start).normalized() else {
        return false;
    };
    let offset = point - axis_start;
    let perpendicular = offset - axis * axis.dot(offset);
    perpendicular.dot(perpendicular) <= EPSILON * EPSILON * 16.0
}
