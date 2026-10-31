//! Authoritative kirigami sheet topology and fold semantics.
//!
//! Rendering, browser interaction, and physical material simulation are consumers of
//! this crate. Cuts and creases are topology operations here, not renderer effects.

mod editing;
mod error;
mod flat_pattern;
mod foldability;
mod folding;
mod geometry;
mod render;
mod self_intersection;
#[cfg(test)]
mod tests;
mod topology;

pub use error::{MeshBuildError, ModelError};
pub use flat_pattern::{
    FlatPatternEdge, FlatPatternEdgeKind, FlatPatternGraph, FlatPatternOperation,
    FlatPatternSnapshot, PatternBounds,
};
pub use foldability::{
    LocalFlatFoldabilityIssue, LocalFlatFoldabilityReport, LocalFlatFoldabilityScope,
};
pub use render::{RenderSeam, RenderSnapshot};
pub use self_intersection::{SelfIntersectionPair, SelfIntersectionReport, SelfIntersectionScope};
pub use topology::{
    FaceBoundaryLoops, FaceId, FaceTriangulation, HalfEdgeId, PlanarTopology, TopologyEdge,
    TopologyError, VertexId,
};

use serde::Serialize;
use std::collections::{HashSet, VecDeque};

const EPSILON: f32 = 1.0e-5;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}

impl Point2 {
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum OperationKind {
    Crease,
    Cut,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct PanelId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct OperationId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct SeamId(pub u32);

#[derive(Debug, Clone, PartialEq)]
pub struct Panel {
    pub id: PanelId,
    pub face: FaceId,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Operation {
    pub id: OperationId,
    pub kind: OperationKind,
    pub path: Vec<Point2>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Seam {
    pub id: SeamId,
    pub operation: OperationId,
    pub kind: OperationKind,
    pub start: Point2,
    pub end: Point2,
    /// Creases connect two panels. A cut boundary bridge can have the same panel on
    /// both sides because it opens material without creating a second panel.
    pub panel_a: PanelId,
    pub panel_b: PanelId,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PaperModel {
    topology: PlanarTopology,
    panels: Vec<Panel>,
    operations: Vec<Operation>,
    seams: Vec<Seam>,
    next_panel_id: u32,
    next_operation_id: u32,
    next_seam_id: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FoldRequest {
    pub operation: OperationId,
    pub angle_radians: f32,
}

impl PaperModel {
    pub fn rectangle(width: f32, height: f32) -> Result<Self, ModelError> {
        if !width.is_finite() || !height.is_finite() || width <= 0.0 || height <= 0.0 {
            return Err(ModelError::InvalidSheetDimensions);
        }
        let half_width = width * 0.5;
        let half_height = height * 0.5;
        let topology = PlanarTopology::from_polygon(vec![
            Point2::new(-half_width, -half_height),
            Point2::new(half_width, -half_height),
            Point2::new(half_width, half_height),
            Point2::new(-half_width, half_height),
        ])?;
        Ok(Self {
            topology,
            panels: vec![Panel {
                id: PanelId(0),
                face: FaceId(0),
            }],
            operations: Vec::new(),
            seams: Vec::new(),
            next_panel_id: 1,
            next_operation_id: 0,
            next_seam_id: 0,
        })
    }

    pub fn topology(&self) -> &PlanarTopology {
        &self.topology
    }

    pub fn panels(&self) -> &[Panel] {
        &self.panels
    }

    pub fn operations(&self) -> &[Operation] {
        &self.operations
    }

    pub fn seams(&self) -> &[Seam] {
        &self.seams
    }

    pub fn component_count(&self) -> usize {
        if self.panels.is_empty() {
            return 0;
        }
        let mut visited = HashSet::new();
        let mut count = 0;
        for panel in &self.panels {
            if visited.contains(&panel.id) {
                continue;
            }
            count += 1;
            let mut queue = VecDeque::from([panel.id]);
            visited.insert(panel.id);
            while let Some(current) = queue.pop_front() {
                for seam in self
                    .seams
                    .iter()
                    .filter(|seam| seam.kind == OperationKind::Crease)
                {
                    let neighbor = adjacent_panel(seam, current);
                    match neighbor {
                        Some(neighbor) if visited.insert(neighbor) => {
                            queue.push_back(neighbor);
                        }
                        _ => {}
                    }
                }
            }
        }
        count
    }
}

fn adjacent_panel(seam: &Seam, panel: PanelId) -> Option<PanelId> {
    if seam.panel_a == panel {
        Some(seam.panel_b)
    } else if seam.panel_b == panel {
        Some(seam.panel_a)
    } else {
        None
    }
}
