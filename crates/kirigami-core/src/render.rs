use crate::{
    FoldRequest, MeshBuildError, ModelError, OperationId, OperationKind, PanelId, PaperModel,
};
use serde::Serialize;
use three_d_core::{Mesh, Vec3};

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RenderSeam {
    pub operation: OperationId,
    pub kind: OperationKind,
    pub start: [f32; 3],
    pub end: [f32; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RenderSnapshot {
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    /// One panel owner per triangle, in the same order as `indices.chunks_exact(3)`.
    pub triangle_panels: Vec<PanelId>,
    pub seams: Vec<RenderSeam>,
    pub panel_count: usize,
    pub component_count: usize,
}

impl PaperModel {
    pub fn render_snapshot(&self, fold: Option<FoldRequest>) -> Result<RenderSnapshot, ModelError> {
        match fold {
            Some(request) => self.render_snapshot_with_folds(&[request]),
            None => self.render_snapshot_with_folds(&[]),
        }
    }

    /// Renders a deterministic multi-crease fold state.
    ///
    /// Fold requests are canonicalized by operation identity so callers cannot change the
    /// result by reordering the same state. Each later hinge is resolved in world space after
    /// the earlier canonical folds. If those folds make a requested straight crease non-rigid
    /// (for example, a crossing crease becomes kinked), the state fails closed rather than
    /// inventing a physically inconsistent axis.
    pub fn render_snapshot_with_folds(
        &self,
        folds: &[FoldRequest],
    ) -> Result<RenderSnapshot, ModelError> {
        let fold_states = self.resolve_folds(folds)?;
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        let mut triangle_panels = Vec::new();

        for panel in &self.panels {
            let triangulation = self.topology.triangulate_face(panel.face)?;
            let panel_start = vertices.len();
            for point in triangulation.vertices {
                let point_3d = Self::transform_point_for_panel(
                    &fold_states,
                    panel.id,
                    Vec3::new(point.x, point.y, 0.0),
                );
                vertices.push([point_3d.x, point_3d.y, point_3d.z]);
            }
            for triangle in triangulation.triangles {
                for local_index in triangle {
                    let global_index = panel_start
                        .checked_add(local_index as usize)
                        .ok_or(ModelError::TooManyRenderVertices)?;
                    indices.push(
                        u32::try_from(global_index)
                            .map_err(|_| ModelError::TooManyRenderVertices)?,
                    );
                }
                triangle_panels.push(panel.id);
            }
        }

        let seams = self
            .seams
            .iter()
            .map(|seam| {
                let start = Self::transform_point_for_panel(
                    &fold_states,
                    seam.panel_a,
                    Vec3::new(seam.start.x, seam.start.y, 0.0),
                );
                let end = Self::transform_point_for_panel(
                    &fold_states,
                    seam.panel_a,
                    Vec3::new(seam.end.x, seam.end.y, 0.0),
                );
                RenderSeam {
                    operation: seam.operation,
                    kind: seam.kind,
                    start: [start.x, start.y, start.z],
                    end: [end.x, end.y, end.z],
                }
            })
            .collect();

        Ok(RenderSnapshot {
            vertices,
            indices,
            triangle_panels,
            seams,
            panel_count: self.panels.len(),
            component_count: self.component_count(),
        })
    }

    pub fn three_d_mesh(&self, fold: Option<FoldRequest>) -> Result<Mesh, MeshBuildError> {
        match fold {
            Some(request) => self.three_d_mesh_with_folds(&[request]),
            None => self.three_d_mesh_with_folds(&[]),
        }
    }

    pub fn three_d_mesh_with_folds(&self, folds: &[FoldRequest]) -> Result<Mesh, MeshBuildError> {
        let snapshot = self
            .render_snapshot_with_folds(folds)
            .map_err(MeshBuildError::Model)?;
        let vertices = snapshot
            .vertices
            .into_iter()
            .map(|[x, y, z]| Vec3::new(x, y, z))
            .collect();
        Mesh::new(vertices, snapshot.indices).map_err(MeshBuildError::Mesh)
    }
}
