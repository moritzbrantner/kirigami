//! Target-geometry ingestion for kirigami approximation.
//!
//! This crate owns Kirigami's normalized target boundary, not paper topology.
//! File-format semantics stay in 3d-lab adapters and browser code only supplies bytes.

use serde::{Deserialize, Serialize};
use std::fmt;
use three_d_animation::{Joint, Mat4, Skeleton, Transform, TransformNode, world_matrices};
use three_d_core::{Mesh, Vec3};
use three_d_formats::{load_gltf, load_obj};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetKind {
    Mesh,
    Skeleton,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TargetGeometry {
    Mesh(MeshTarget),
    Skeleton(SkeletonTarget),
}

#[derive(Debug, Clone, PartialEq)]
pub struct MeshTarget {
    meshes: Vec<Mesh>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SkeletonTarget {
    skeleton: Skeleton,
    joint_names: Vec<Option<String>>,
    world_positions: Vec<Vec3>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TargetSnapshot {
    pub kind: TargetKind,
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    pub edges: Vec<[u32; 2]>,
    pub joint_names: Vec<Option<String>>,
    pub mesh_count: usize,
    pub triangle_count: usize,
    pub joint_count: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TargetError {
    UnsupportedFileType,
    MeshFormat(String),
    InvalidSkeletonJson(String),
    EmptySkeleton,
    NonFiniteJointTranslation { joint: usize },
    InvalidSkeleton(String),
    TooManyVertices,
}

impl fmt::Display for TargetError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedFileType => formatter
                .write_str("supported target files are .obj, .gltf, .glb, and skeleton .json"),
            Self::MeshFormat(error) => write!(formatter, "could not load target mesh: {error}"),
            Self::InvalidSkeletonJson(error) => {
                write!(formatter, "invalid skeleton JSON: {error}")
            }
            Self::EmptySkeleton => formatter.write_str("a skeleton requires at least one joint"),
            Self::NonFiniteJointTranslation { joint } => {
                write!(
                    formatter,
                    "skeleton joint {joint} has a non-finite translation"
                )
            }
            Self::InvalidSkeleton(error) => write!(formatter, "invalid skeleton: {error}"),
            Self::TooManyVertices => {
                formatter.write_str("target exceeds the u32 preview index capacity")
            }
        }
    }
}

impl std::error::Error for TargetError {}

#[derive(Debug, Deserialize)]
struct SkeletonDocument {
    joints: Vec<SkeletonJointDocument>,
}

#[derive(Debug, Deserialize)]
struct SkeletonJointDocument {
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    parent: Option<usize>,
    translation: [f32; 3],
}

impl TargetGeometry {
    pub fn snapshot(&self) -> Result<TargetSnapshot, TargetError> {
        match self {
            Self::Mesh(target) => target.snapshot(),
            Self::Skeleton(target) => target.snapshot(),
        }
    }
}

impl MeshTarget {
    fn from_meshes(meshes: Vec<Mesh>) -> Self {
        Self { meshes }
    }

    fn snapshot(&self) -> Result<TargetSnapshot, TargetError> {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        for mesh in &self.meshes {
            let base = u32::try_from(vertices.len()).map_err(|_| TargetError::TooManyVertices)?;
            vertices.extend(
                mesh.vertices()
                    .iter()
                    .map(|vertex| [vertex.x, vertex.y, vertex.z]),
            );
            for index in mesh.indices() {
                indices.push(
                    base.checked_add(*index)
                        .ok_or(TargetError::TooManyVertices)?,
                );
            }
        }

        Ok(TargetSnapshot {
            kind: TargetKind::Mesh,
            triangle_count: indices.len() / 3,
            vertices,
            indices,
            edges: Vec::new(),
            joint_names: Vec::new(),
            mesh_count: self.meshes.len(),
            joint_count: 0,
        })
    }
}

impl SkeletonTarget {
    fn from_document(document: SkeletonDocument) -> Result<Self, TargetError> {
        if document.joints.is_empty() {
            return Err(TargetError::EmptySkeleton);
        }

        for (joint, item) in document.joints.iter().enumerate() {
            if item.translation.iter().any(|value| !value.is_finite()) {
                return Err(TargetError::NonFiniteJointTranslation { joint });
            }
        }

        let skeleton = Skeleton::new(
            document
                .joints
                .iter()
                .map(|joint| Joint {
                    parent: joint.parent,
                    inverse_bind: Mat4::IDENTITY,
                })
                .collect(),
        )
        .map_err(|error| TargetError::InvalidSkeleton(error.to_string()))?;

        let nodes = document
            .joints
            .iter()
            .map(|joint| TransformNode {
                parent: joint.parent,
                local: Transform {
                    translation: Vec3::new(
                        joint.translation[0],
                        joint.translation[1],
                        joint.translation[2],
                    ),
                    ..Transform::default()
                },
            })
            .collect::<Vec<_>>();
        let world = world_matrices(&nodes)
            .map_err(|error| TargetError::InvalidSkeleton(error.to_string()))?;
        let world_positions = world
            .into_iter()
            .map(|matrix| matrix.transform_point(Vec3::ZERO))
            .collect();

        Ok(Self {
            skeleton,
            joint_names: document
                .joints
                .into_iter()
                .map(|joint| joint.name)
                .collect(),
            world_positions,
        })
    }

    fn snapshot(&self) -> Result<TargetSnapshot, TargetError> {
        let vertices = self
            .world_positions
            .iter()
            .map(|position| [position.x, position.y, position.z])
            .collect::<Vec<_>>();
        let mut edges = Vec::new();

        for (joint_index, joint) in self.skeleton.joints().iter().enumerate() {
            if let Some(parent) = joint.parent {
                edges.push([
                    u32::try_from(parent).map_err(|_| TargetError::TooManyVertices)?,
                    u32::try_from(joint_index).map_err(|_| TargetError::TooManyVertices)?,
                ]);
            }
        }

        Ok(TargetSnapshot {
            kind: TargetKind::Skeleton,
            vertices,
            indices: Vec::new(),
            edges,
            joint_names: self.joint_names.clone(),
            mesh_count: 0,
            triangle_count: 0,
            joint_count: self.skeleton.joints().len(),
        })
    }
}

pub fn load_target(file_name: &str, bytes: &[u8]) -> Result<TargetGeometry, TargetError> {
    let extension = file_name
        .rsplit_once('.')
        .map(|(_, extension)| extension.to_ascii_lowercase())
        .ok_or(TargetError::UnsupportedFileType)?;

    match extension.as_str() {
        "obj" => load_mesh_target(load_obj(bytes).map_err(mesh_format_error)?),
        "gltf" | "glb" => load_mesh_target(load_gltf(bytes).map_err(mesh_format_error)?),
        "json" => {
            let document: SkeletonDocument = serde_json::from_slice(bytes)
                .map_err(|error| TargetError::InvalidSkeletonJson(error.to_string()))?;
            Ok(TargetGeometry::Skeleton(SkeletonTarget::from_document(
                document,
            )?))
        }
        _ => Err(TargetError::UnsupportedFileType),
    }
}

fn load_mesh_target(asset: three_d_assets::Asset) -> Result<TargetGeometry, TargetError> {
    let meshes = asset
        .meshes()
        .iter()
        .flat_map(|mesh| mesh.primitives())
        .map(|primitive| primitive.mesh().clone())
        .collect::<Vec<_>>();
    Ok(TargetGeometry::Mesh(MeshTarget::from_meshes(meshes)))
}

fn mesh_format_error(error: impl fmt::Display) -> TargetError {
    TargetError::MeshFormat(error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_obj_into_renderer_neutral_mesh_snapshot() {
        let target = load_target("triangle.obj", b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").unwrap();
        let snapshot = target.snapshot().unwrap();

        assert_eq!(snapshot.kind, TargetKind::Mesh);
        assert_eq!(snapshot.mesh_count, 1);
        assert_eq!(snapshot.triangle_count, 1);
        assert_eq!(snapshot.vertices.len(), 3);
        assert_eq!(snapshot.indices.len(), 3);
    }

    #[test]
    fn skeleton_translations_are_resolved_through_shared_hierarchy_semantics() {
        let target = load_target(
            "figure.json",
            br#"{"joints":[{"name":"root","translation":[1.0,0.0,0.0]},{"name":"tip","parent":0,"translation":[0.0,2.0,0.0]}]}"#,
        )
        .unwrap();
        let snapshot = target.snapshot().unwrap();

        assert_eq!(snapshot.kind, TargetKind::Skeleton);
        assert_eq!(snapshot.joint_count, 2);
        assert_eq!(snapshot.edges, vec![[0, 1]]);
        assert_eq!(snapshot.vertices[0], [1.0, 0.0, 0.0]);
        assert_eq!(snapshot.vertices[1], [1.0, 2.0, 0.0]);
    }

    #[test]
    fn skeleton_parent_order_is_fail_closed() {
        let error = load_target(
            "invalid.json",
            br#"{"joints":[{"parent":1,"translation":[0.0,0.0,0.0]},{"translation":[0.0,1.0,0.0]}]}"#,
        )
        .unwrap_err();

        assert!(matches!(error, TargetError::InvalidSkeleton(_)));
    }
}
