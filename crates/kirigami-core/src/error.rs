use crate::topology::TopologyError;
use crate::{OperationId, PanelId, SeamId};
use std::fmt;
use three_d_core::MeshError;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ModelError {
    InvalidSheetDimensions,
    NonFinitePoint,
    DegenerateSegment,
    InvalidPath,
    BentCreaseRequiresConstraintSolver,
    DuplicateFoldOperation(OperationId),
    FoldConstraintConflict(OperationId),
    UnknownPanel(PanelId),
    UnknownOperation(OperationId),
    SegmentEndpointOffBoundary,
    SegmentDoesNotSplitPanel,
    CannotFoldCut(OperationId),
    NonFiniteFoldAngle,
    SeamReattachmentFailed(SeamId),
    Topology(TopologyError),
    TooManyRenderVertices,
    TooManyCollisionTriangles,
}

impl fmt::Display for ModelError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSheetDimensions => {
                formatter.write_str("sheet dimensions must be finite and positive")
            }
            Self::NonFinitePoint => {
                formatter.write_str("segment points must contain only finite coordinates")
            }
            Self::DegenerateSegment => formatter.write_str("segment endpoints must be distinct"),
            Self::InvalidPath => formatter.write_str("a path requires at least two finite points"),
            Self::BentCreaseRequiresConstraintSolver => formatter
                .write_str("a non-straight crease requires the multi-crease constraint solver"),
            Self::DuplicateFoldOperation(operation) => write!(
                formatter,
                "fold operation {} was requested more than once",
                operation.0
            ),
            Self::FoldConstraintConflict(operation) => write!(
                formatter,
                "fold operation {} cannot be satisfied by the current rigid crease state",
                operation.0
            ),
            Self::UnknownPanel(panel) => write!(formatter, "unknown panel {}", panel.0),
            Self::UnknownOperation(operation) => {
                write!(formatter, "unknown operation {}", operation.0)
            }
            Self::SegmentEndpointOffBoundary => formatter
                .write_str("both segment endpoints must lie on the selected panel boundary"),
            Self::SegmentDoesNotSplitPanel => formatter
                .write_str("segment does not split the selected panel into two valid faces"),
            Self::CannotFoldCut(operation) => write!(
                formatter,
                "operation {} is a cut and cannot be folded",
                operation.0
            ),
            Self::NonFiniteFoldAngle => formatter.write_str("fold angle must be finite"),
            Self::SeamReattachmentFailed(seam) => write!(
                formatter,
                "could not reattach seam {} after splitting its incident panel",
                seam.0
            ),
            Self::Topology(error) => write!(formatter, "topology error: {error}"),
            Self::TooManyRenderVertices => {
                formatter.write_str("render snapshot exceeds u32 index capacity")
            }
            Self::TooManyCollisionTriangles => {
                formatter.write_str("collision query exceeds u32 triangle-ID capacity")
            }
        }
    }
}

impl std::error::Error for ModelError {}

impl From<TopologyError> for ModelError {
    fn from(error: TopologyError) -> Self {
        Self::Topology(error)
    }
}

#[derive(Debug)]
pub enum MeshBuildError {
    Model(ModelError),
    Mesh(MeshError),
}

impl fmt::Display for MeshBuildError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Model(error) => write!(formatter, "{error}"),
            Self::Mesh(error) => write!(
                formatter,
                "3d-lab mesh rejected kirigami geometry: {error:?}"
            ),
        }
    }
}

impl std::error::Error for MeshBuildError {}
