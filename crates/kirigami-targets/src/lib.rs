//! Target-geometry ingestion and normalization for kirigami approximation.
//!
//! This crate owns Kirigami's normalized target boundary, not paper topology.
//! File-format semantics stay in 3d-lab adapters and browser code only supplies bytes.

use serde::{Deserialize, Serialize};
use std::fmt;
use three_d_animation::{Joint, Mat4, Skeleton, Transform, TransformNode, world_matrices};
use three_d_core::{Mesh, Vec3};
use three_d_formats::{load_gltf, load_obj};

const NORMALIZED_MAX_EXTENT: f32 = 1.0;
const NORMALIZATION_EPSILON: f32 = 1.0e-6;
const FNV1A64_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV1A64_PRIME: u64 = 0x100000001b3;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetKind {
    Mesh,
    Skeleton,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetNormalizationRule {
    CenteredUnitMaxExtent,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetOrientationRule {
    PreserveSourceAxes,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct TargetBounds {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct TargetNormalization {
    pub rule: TargetNormalizationRule,
    pub orientation_rule: TargetOrientationRule,
    pub source_bounds: TargetBounds,
    pub source_center: [f32; 3],
    pub source_max_extent: f32,
    pub uniform_scale: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct TargetMeasurements {
    pub mesh_surface_area: Option<f32>,
    pub skeleton_total_edge_length: Option<f32>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TargetGeometry {
    Mesh(MeshTarget),
    Skeleton(SkeletonTarget),
}

#[derive(Debug, Clone, PartialEq)]
pub struct MeshTarget {
    meshes: Vec<Mesh>,
    source: TargetSourceReceipt,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SkeletonTarget {
    skeleton: Skeleton,
    joint_names: Vec<Option<String>>,
    world_positions: Vec<Vec3>,
    source: TargetSourceReceipt,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TargetSnapshot {
    pub kind: TargetKind,
    /// Vertices are centered and uniformly scaled so the largest source extent is 1.
    /// Source axes are deliberately preserved until an explicit orientation policy exists.
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    pub edges: Vec<[u32; 2]>,
    pub joint_names: Vec<Option<String>>,
    pub mesh_count: usize,
    pub triangle_count: usize,
    pub joint_count: usize,
    pub source_fingerprint: String,
    pub source_byte_length: usize,
    pub normalization: TargetNormalization,
    pub measurements: TargetMeasurements,
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
            Self::NonFiniteJointTranslation { joint } => write!(
                formatter,
                "skeleton joint {joint} has a non-finite translation"
            ),
            Self::InvalidSkeleton(error) => write!(formatter, "invalid skeleton: {error}"),
            Self::TooManyVertices => {
                formatter.write_str("target exceeds the u32 preview index capacity")
            }
        }
    }
}

impl std::error::Error for TargetError {}

#[derive(Debug, Clone, PartialEq)]
struct TargetSourceReceipt {
    fingerprint: String,
    byte_length: usize,
}

impl TargetSourceReceipt {
    fn from_bytes(bytes: &[u8]) -> Self {
        Self {
            fingerprint: stable_source_fingerprint(bytes),
            byte_length: bytes.len(),
        }
    }
}

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
    fn from_meshes(meshes: Vec<Mesh>, source: TargetSourceReceipt) -> Self {
        Self { meshes, source }
    }

    fn snapshot(&self) -> Result<TargetSnapshot, TargetError> {
        let mut source_vertices = Vec::new();
        let mut indices = Vec::new();

        for mesh in &self.meshes {
            let base =
                u32::try_from(source_vertices.len()).map_err(|_| TargetError::TooManyVertices)?;
            source_vertices.extend(
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

        let normalization = TargetNormalization::from_points(&source_vertices);
        let vertices = normalize_points(&source_vertices, normalization);
        let mesh_surface_area = triangle_surface_area(&vertices, &indices);

        Ok(TargetSnapshot {
            kind: TargetKind::Mesh,
            triangle_count: indices.len() / 3,
            vertices,
            indices,
            edges: Vec::new(),
            joint_names: Vec::new(),
            mesh_count: self.meshes.len(),
            joint_count: 0,
            source_fingerprint: self.source.fingerprint.clone(),
            source_byte_length: self.source.byte_length,
            normalization,
            measurements: TargetMeasurements {
                mesh_surface_area: Some(mesh_surface_area),
                skeleton_total_edge_length: None,
            },
        })
    }
}

impl SkeletonTarget {
    fn from_document(
        document: SkeletonDocument,
        source: TargetSourceReceipt,
    ) -> Result<Self, TargetError> {
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
            source,
        })
    }

    fn snapshot(&self) -> Result<TargetSnapshot, TargetError> {
        let source_vertices = self
            .world_positions
            .iter()
            .map(|position| [position.x, position.y, position.z])
            .collect::<Vec<_>>();
        let normalization = TargetNormalization::from_points(&source_vertices);
        let vertices = normalize_points(&source_vertices, normalization);
        let mut edges = Vec::new();

        for (joint_index, joint) in self.skeleton.joints().iter().enumerate() {
            if let Some(parent) = joint.parent {
                edges.push([
                    u32::try_from(parent).map_err(|_| TargetError::TooManyVertices)?,
                    u32::try_from(joint_index).map_err(|_| TargetError::TooManyVertices)?,
                ]);
            }
        }
        let skeleton_total_edge_length = total_edge_length(&vertices, &edges);

        Ok(TargetSnapshot {
            kind: TargetKind::Skeleton,
            vertices,
            indices: Vec::new(),
            edges,
            joint_names: self.joint_names.clone(),
            mesh_count: 0,
            triangle_count: 0,
            joint_count: self.skeleton.joints().len(),
            source_fingerprint: self.source.fingerprint.clone(),
            source_byte_length: self.source.byte_length,
            normalization,
            measurements: TargetMeasurements {
                mesh_surface_area: None,
                skeleton_total_edge_length: Some(skeleton_total_edge_length),
            },
        })
    }
}

impl TargetNormalization {
    fn from_points(points: &[[f32; 3]]) -> Self {
        let Some(first) = points.first().copied() else {
            return Self {
                rule: TargetNormalizationRule::CenteredUnitMaxExtent,
                orientation_rule: TargetOrientationRule::PreserveSourceAxes,
                source_bounds: TargetBounds {
                    min: [0.0; 3],
                    max: [0.0; 3],
                },
                source_center: [0.0; 3],
                source_max_extent: 0.0,
                uniform_scale: 1.0,
            };
        };

        let mut min = first;
        let mut max = first;
        for point in &points[1..] {
            for axis in 0..3 {
                min[axis] = min[axis].min(point[axis]);
                max[axis] = max[axis].max(point[axis]);
            }
        }

        let source_center =
            std::array::from_fn(|axis| (min[axis] + max[axis]) * 0.5);
        let source_max_extent = (0..3)
            .map(|axis| max[axis] - min[axis])
            .fold(0.0_f32, f32::max);
        let uniform_scale = if source_max_extent > NORMALIZATION_EPSILON {
            NORMALIZED_MAX_EXTENT / source_max_extent
        } else {
            1.0
        };

        Self {
            rule: TargetNormalizationRule::CenteredUnitMaxExtent,
            orientation_rule: TargetOrientationRule::PreserveSourceAxes,
            source_bounds: TargetBounds { min, max },
            source_center,
            source_max_extent,
            uniform_scale,
        }
    }
}

pub fn load_target(file_name: &str, bytes: &[u8]) -> Result<TargetGeometry, TargetError> {
    let extension = file_name
        .rsplit_once('.')
        .map(|(_, extension)| extension.to_ascii_lowercase())
        .ok_or(TargetError::UnsupportedFileType)?;
    let source = TargetSourceReceipt::from_bytes(bytes);

    match extension.as_str() {
        "obj" => load_mesh_target(load_obj(bytes).map_err(mesh_format_error)?, source),
        "gltf" | "glb" => load_mesh_target(load_gltf(bytes).map_err(mesh_format_error)?, source),
        "json" => {
            let document: SkeletonDocument = serde_json::from_slice(bytes)
                .map_err(|error| TargetError::InvalidSkeletonJson(error.to_string()))?;
            Ok(TargetGeometry::Skeleton(SkeletonTarget::from_document(
                document, source,
            )?))
        }
        _ => Err(TargetError::UnsupportedFileType),
    }
}

fn load_mesh_target(
    asset: three_d_assets::Asset,
    source: TargetSourceReceipt,
) -> Result<TargetGeometry, TargetError> {
    let meshes = asset
        .meshes()
        .iter()
        .flat_map(|mesh| mesh.primitives())
        .map(|primitive| primitive.mesh().clone())
        .collect::<Vec<_>>();
    Ok(TargetGeometry::Mesh(MeshTarget::from_meshes(
        meshes, source,
    )))
}

fn normalize_points(
    points: &[[f32; 3]],
    normalization: TargetNormalization,
) -> Vec<[f32; 3]> {
    points
        .iter()
        .map(|point| {
            std::array::from_fn(|axis| {
                (point[axis] - normalization.source_center[axis])
                    * normalization.uniform_scale
            })
        })
        .collect()
}

fn triangle_surface_area(vertices: &[[f32; 3]], indices: &[u32]) -> f32 {
    indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = vertices[triangle[0] as usize];
            let b = vertices[triangle[1] as usize];
            let c = vertices[triangle[2] as usize];
            let ab = subtract(b, a);
            let ac = subtract(c, a);
            0.5 * length(cross(ab, ac))
        })
        .sum()
}

fn total_edge_length(vertices: &[[f32; 3]], edges: &[[u32; 2]]) -> f32 {
    edges
        .iter()
        .map(|edge| {
            length(subtract(
                vertices[edge[1] as usize],
                vertices[edge[0] as usize],
            ))
        })
        .sum()
}

fn subtract(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|axis| left[axis] - right[axis])
}

fn cross(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn length(vector: [f32; 3]) -> f32 {
    vector.iter().map(|value| value * value).sum::<f32>().sqrt()
}

fn stable_source_fingerprint(bytes: &[u8]) -> String {
    let hash = bytes.iter().fold(FNV1A64_OFFSET_BASIS, |hash, byte| {
        (hash ^ u64::from(*byte)).wrapping_mul(FNV1A64_PRIME)
    });
    format!("fnv1a64:{hash:016x}")
}

fn mesh_format_error(error: impl fmt::Display) -> TargetError {
    TargetError::MeshFormat(error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalization_centers_and_scales_the_largest_extent() {
        let normalization =
            TargetNormalization::from_points(&[[2.0, 4.0, -2.0], [6.0, 6.0, 0.0]]);
        let normalized =
            normalize_points(&[[2.0, 4.0, -2.0], [6.0, 6.0, 0.0]], normalization);

        assert_eq!(normalization.source_center, [4.0, 5.0, -1.0]);
        assert_eq!(normalization.source_max_extent, 4.0);
        assert_eq!(normalization.uniform_scale, 0.25);
        assert_eq!(normalized, vec![[-0.5, -0.25, -0.25], [0.5, 0.25, 0.25]]);
    }

    #[test]
    fn degenerate_points_remain_finite_at_the_origin() {
        let normalization = TargetNormalization::from_points(&[[3.0, -2.0, 8.0]]);
        let normalized = normalize_points(&[[3.0, -2.0, 8.0]], normalization);

        assert_eq!(normalization.uniform_scale, 1.0);
        assert_eq!(normalized, vec![[0.0, 0.0, 0.0]]);
    }

    #[test]
    fn fingerprint_is_stable_for_exact_bytes() {
        assert_eq!(
            stable_source_fingerprint(b"kirigami"),
            stable_source_fingerprint(b"kirigami")
        );
        assert_ne!(
            stable_source_fingerprint(b"kirigami"),
            stable_source_fingerprint(b"Kirigami")
        );
    }
}
