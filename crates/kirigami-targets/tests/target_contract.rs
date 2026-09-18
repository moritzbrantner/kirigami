use kirigami_targets::{
    TargetError, TargetKind, TargetNormalizationRule, TargetOrientationRule, load_target,
};

const TRIANGLE_GLTF: &[u8] = br#"{
  "asset": {"version": "2.0"},
  "buffers": [{
    "byteLength": 42,
    "uri": "data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAAAAQAAAAAAAAAAAAAAAAAAAAEAAAAAAAAABAAIA"
  }],
  "bufferViews": [
    {"buffer": 0, "byteOffset": 0, "byteLength": 36},
    {"buffer": 0, "byteOffset": 36, "byteLength": 6}
  ],
  "accessors": [
    {
      "bufferView": 0,
      "componentType": 5126,
      "count": 3,
      "type": "VEC3",
      "min": [0.0, 0.0, 0.0],
      "max": [2.0, 2.0, 0.0]
    },
    {
      "bufferView": 1,
      "componentType": 5123,
      "count": 3,
      "type": "SCALAR"
    }
  ],
  "meshes": [{
    "primitives": [{
      "attributes": {"POSITION": 0},
      "indices": 1
    }]
  }]
}"#;

fn assert_close(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 1.0e-5,
        "expected {expected}, got {actual}"
    );
}

fn assert_vertices_close(actual: &[[f32; 3]], expected: &[[f32; 3]]) {
    assert_eq!(actual.len(), expected.len());
    for (actual, expected) in actual.iter().zip(expected) {
        for axis in 0..3 {
            assert_close(actual[axis], expected[axis]);
        }
    }
}

#[test]
fn equivalent_obj_geometry_normalizes_identically_across_translation_and_scale() {
    let first = load_target(
        "first.obj",
        b"v 0 0 0\nv 2 0 0\nv 0 2 0\nf 1 2 3\n",
    )
    .unwrap()
    .snapshot()
    .unwrap();
    let second = load_target(
        "second.obj",
        b"v 10 -4 2\nv 18 -4 2\nv 10 4 2\nf 1 2 3\n",
    )
    .unwrap()
    .snapshot()
    .unwrap();

    assert_eq!(first.kind, TargetKind::Mesh);
    assert_eq!(first.indices, second.indices);
    assert_vertices_close(&first.vertices, &second.vertices);
    assert_eq!(
        first.normalization.rule,
        TargetNormalizationRule::CenteredUnitMaxExtent
    );
    assert_eq!(
        first.normalization.orientation_rule,
        TargetOrientationRule::PreserveSourceAxes
    );
    assert_close(first.measurements.mesh_surface_area.unwrap(), 0.5);
    assert_close(second.measurements.mesh_surface_area.unwrap(), 0.5);
}

#[test]
fn exact_source_fingerprint_changes_even_when_normalized_geometry_is_equivalent() {
    let plain = load_target(
        "triangle.obj",
        b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    )
    .unwrap()
    .snapshot()
    .unwrap();
    let commented = load_target(
        "triangle.obj",
        b"# same geometry, different source bytes\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
    )
    .unwrap()
    .snapshot()
    .unwrap();

    assert_vertices_close(&plain.vertices, &commented.vertices);
    assert_ne!(plain.source_fingerprint, commented.source_fingerprint);
    assert_ne!(plain.source_byte_length, commented.source_byte_length);
    assert!(plain.source_fingerprint.starts_with("fnv1a64:"));
}

#[test]
fn embedded_gltf_loads_through_the_shared_format_adapter() {
    let snapshot = load_target("triangle.GLTF", TRIANGLE_GLTF)
        .unwrap()
        .snapshot()
        .unwrap();

    assert_eq!(snapshot.kind, TargetKind::Mesh);
    assert_eq!(snapshot.mesh_count, 1);
    assert_eq!(snapshot.triangle_count, 1);
    assert_eq!(snapshot.indices, vec![0, 1, 2]);
    assert_close(snapshot.normalization.source_max_extent, 2.0);
}

#[test]
fn glb_extension_loads_binary_gltf() {
    let bytes = minimal_glb();
    let snapshot = load_target("triangle.glb", &bytes)
        .unwrap()
        .snapshot()
        .unwrap();

    assert_eq!(snapshot.kind, TargetKind::Mesh);
    assert_eq!(snapshot.triangle_count, 1);
    assert_eq!(snapshot.vertices.len(), 3);
    assert_eq!(snapshot.source_byte_length, bytes.len());
}

#[test]
fn skeleton_hierarchy_is_normalized_without_losing_edges_or_names() {
    let snapshot = load_target(
        "person.json",
        br#"{
          "joints": [
            {"name": "root", "translation": [10.0, 0.0, 0.0]},
            {"name": "spine", "parent": 0, "translation": [0.0, 4.0, 0.0]},
            {"name": "head", "parent": 1, "translation": [0.0, 2.0, 0.0]}
          ]
        }"#,
    )
    .unwrap()
    .snapshot()
    .unwrap();

    assert_eq!(snapshot.kind, TargetKind::Skeleton);
    assert_eq!(snapshot.joint_names, vec![
        Some("root".to_owned()),
        Some("spine".to_owned()),
        Some("head".to_owned()),
    ]);
    assert_eq!(snapshot.edges, vec![[0, 1], [1, 2]]);
    assert_vertices_close(
        &snapshot.vertices,
        &[[0.0, -0.5, 0.0], [0.0, 1.0 / 6.0, 0.0], [0.0, 0.5, 0.0]],
    );
    assert_close(
        snapshot.measurements.skeleton_total_edge_length.unwrap(),
        1.0,
    );
}

#[test]
fn one_joint_skeleton_remains_finite_and_centered() {
    let snapshot = load_target(
        "point.json",
        br#"{"joints":[{"name":"root","translation":[7.0,-3.0,2.0]}]}"#,
    )
    .unwrap()
    .snapshot()
    .unwrap();

    assert_eq!(snapshot.vertices, vec![[0.0, 0.0, 0.0]]);
    assert_eq!(snapshot.normalization.source_max_extent, 0.0);
    assert_eq!(snapshot.normalization.uniform_scale, 1.0);
    assert_eq!(
        snapshot.measurements.skeleton_total_edge_length,
        Some(0.0)
    );
}

#[test]
fn invalid_inputs_fail_closed_at_the_target_boundary() {
    assert!(matches!(
        load_target("target.txt", b"not a target"),
        Err(TargetError::UnsupportedFileType)
    ));
    assert!(matches!(
        load_target("target.json", br#"{"joints":[]}"#),
        Err(TargetError::EmptySkeleton)
    ));
    assert!(matches!(
        load_target(
            "target.json",
            br#"{"joints":[{"parent":1,"translation":[0.0,0.0,0.0]},{"translation":[0.0,1.0,0.0]}]}"#
        ),
        Err(TargetError::InvalidSkeleton(_))
    ));
    assert!(matches!(
        load_target("target.json", b"{"),
        Err(TargetError::InvalidSkeletonJson(_))
    ));
}

fn minimal_glb() -> Vec<u8> {
    let json = br#"{
      "asset":{"version":"2.0"},
      "buffers":[{"byteLength":42}],
      "bufferViews":[
        {"buffer":0,"byteOffset":0,"byteLength":36},
        {"buffer":0,"byteOffset":36,"byteLength":6}
      ],
      "accessors":[
        {"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[2,2,0]},
        {"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}
      ],
      "meshes":[{"primitives":[{"attributes":{"POSITION":0},"indices":1}]}]
    }"#;

    let mut json_chunk = json.to_vec();
    while !json_chunk.len().is_multiple_of(4) {
        json_chunk.push(b' ');
    }

    let mut bin_chunk = Vec::new();
    for value in [0.0_f32, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0, 0.0] {
        bin_chunk.extend_from_slice(&value.to_le_bytes());
    }
    for index in [0_u16, 1, 2] {
        bin_chunk.extend_from_slice(&index.to_le_bytes());
    }
    while !bin_chunk.len().is_multiple_of(4) {
        bin_chunk.push(0);
    }

    let total_length = 12 + 8 + json_chunk.len() + 8 + bin_chunk.len();
    let mut glb = Vec::with_capacity(total_length);
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2_u32.to_le_bytes());
    glb.extend_from_slice(&(total_length as u32).to_le_bytes());
    glb.extend_from_slice(&(json_chunk.len() as u32).to_le_bytes());
    glb.extend_from_slice(&0x4E4F534A_u32.to_le_bytes());
    glb.extend_from_slice(&json_chunk);
    glb.extend_from_slice(&(bin_chunk.len() as u32).to_le_bytes());
    glb.extend_from_slice(&0x004E4942_u32.to_le_bytes());
    glb.extend_from_slice(&bin_chunk);
    glb
}
