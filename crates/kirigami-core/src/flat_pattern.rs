use crate::geometry::{interpolate, point_on_segment};
use crate::topology::{TopologyError, VertexId};
use crate::{ModelError, OperationId, OperationKind, PaperModel, Point2};
use serde::Serialize;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct PatternBounds {
    pub min: Point2,
    pub max: Point2,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FlatPatternOperation {
    pub id: OperationId,
    pub kind: OperationKind,
    pub path: Vec<Point2>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum FlatPatternEdgeKind {
    Boundary,
    Cut,
    Crease,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct FlatPatternEdge {
    /// Indices into `FlatPatternGraph::vertices`.
    pub vertices: [u32; 2],
    pub kind: FlatPatternEdgeKind,
    /// Owning logical operation; `None` for the external material boundary.
    pub operation: Option<OperationId>,
}

/// Planar graph of the flat pattern derived from the authoritative topology, so
/// operation junctions and snapped endpoints share one vertex identity.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FlatPatternGraph {
    pub vertices: Vec<Point2>,
    pub edges: Vec<FlatPatternEdge>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FlatPatternSnapshot {
    /// External material boundary. Cut and crease paths remain separate semantic
    /// operations so exporters never infer paper meaning from topology adjacency.
    pub boundary_segments: Vec<[Point2; 2]>,
    /// Topology-subdivided boundary/cut/crease graph. Unlike logical operation
    /// paths, it contains every junction vertex needed by graph interchange.
    pub graph: FlatPatternGraph,
    pub operations: Vec<FlatPatternOperation>,
    pub bounds: PatternBounds,
}

impl PaperModel {
    pub fn flat_pattern_snapshot(&self) -> Result<FlatPatternSnapshot, ModelError> {
        let boundary_segments = self.topology.external_boundary_segments();
        let first = boundary_segments
            .first()
            .map(|segment| segment[0])
            .ok_or(ModelError::Topology(TopologyError::InvalidTopology))?;
        let mut min = first;
        let mut max = first;
        for segment in &boundary_segments {
            for point in segment {
                min.x = min.x.min(point.x);
                min.y = min.y.min(point.y);
                max.x = max.x.max(point.x);
                max.y = max.y.max(point.y);
            }
        }

        Ok(FlatPatternSnapshot {
            boundary_segments,
            graph: self.flat_pattern_graph()?,
            operations: self
                .operations
                .iter()
                .map(|operation| FlatPatternOperation {
                    id: operation.id,
                    kind: operation.kind,
                    path: operation.path.clone(),
                })
                .collect(),
            bounds: PatternBounds { min, max },
        })
    }

    fn flat_pattern_graph(&self) -> Result<FlatPatternGraph, ModelError> {
        let invalid = || ModelError::Topology(TopologyError::InvalidTopology);
        let mut vertex_indices = std::collections::HashMap::<VertexId, u32>::new();
        let mut vertices = Vec::new();
        let mut edges = Vec::new();
        for edge in self.topology.undirected_edges() {
            let start = self.topology.vertex_point(edge.start).ok_or_else(invalid)?;
            let end = self.topology.vertex_point(edge.end).ok_or_else(invalid)?;
            let (kind, operation) = if edge.external_boundary {
                (FlatPatternEdgeKind::Boundary, None)
            } else {
                let midpoint = interpolate(start, end, 0.5);
                let seam = self
                    .seams
                    .iter()
                    .find(|seam| point_on_segment(midpoint, seam.start, seam.end))
                    .ok_or_else(invalid)?;
                let kind = match seam.kind {
                    OperationKind::Cut => FlatPatternEdgeKind::Cut,
                    OperationKind::Crease => FlatPatternEdgeKind::Crease,
                };
                (kind, Some(seam.operation))
            };
            let mut index_of = |id: VertexId, point: Point2| -> Result<u32, ModelError> {
                if let Some(index) = vertex_indices.get(&id) {
                    return Ok(*index);
                }
                let index = u32::try_from(vertices.len()).map_err(|_| invalid())?;
                vertices.push(point);
                vertex_indices.insert(id, index);
                Ok(index)
            };
            let start = index_of(edge.start, start)?;
            let end = index_of(edge.end, end)?;
            edges.push(FlatPatternEdge {
                vertices: [start, end],
                kind,
                operation,
            });
        }
        Ok(FlatPatternGraph { vertices, edges })
    }
}
