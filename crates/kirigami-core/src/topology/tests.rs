use super::geometry::orientation;
use super::*;

#[test]
fn half_edge_split_preserves_invariants_and_shared_twins() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-1.0, -0.5),
        Point2::new(1.0, -0.5),
        Point2::new(1.0, 0.5),
        Point2::new(-1.0, 0.5),
    ])
    .unwrap();
    let second = topology
        .split_face_with_segment(FaceId(0), Point2::new(0.0, -0.5), Point2::new(0.0, 0.5))
        .unwrap();
    topology
        .split_face_with_segment(FaceId(0), Point2::new(-1.0, 0.0), Point2::new(0.0, 0.0))
        .unwrap();

    topology.validate().unwrap();
    assert_eq!(second, FaceId(1));
    assert_eq!(topology.face_count(), 3);
    assert_eq!(topology.vertex_count(), 8);
    assert!(topology.half_edges.iter().any(|edge| edge.twin.is_some()));
}

#[test]
fn ear_clipping_triangulates_concave_face_without_crossing_boundary() {
    let topology = PlanarTopology::from_polygon(vec![
        Point2::new(0.0, 0.0),
        Point2::new(2.0, 0.0),
        Point2::new(2.0, 1.0),
        Point2::new(1.0, 0.4),
        Point2::new(0.0, 1.0),
    ])
    .unwrap();
    let triangulation = topology.triangulate_face(FaceId(0)).unwrap();
    assert_eq!(triangulation.triangles.len(), 3);
}

#[test]
fn accepts_small_non_degenerate_face() {
    let topology = PlanarTopology::from_polygon(vec![
        Point2::new(0.0, 0.0),
        Point2::new(0.001, 0.0),
        Point2::new(0.001, 0.001),
        Point2::new(0.0, 0.001),
    ])
    .unwrap();
    assert_eq!(topology.face_count(), 1);
    assert_eq!(
        topology
            .triangulate_face(FaceId(0))
            .unwrap()
            .triangles
            .len(),
        2
    );
    topology.validate().unwrap();
}

#[test]
fn allows_chord_continuing_past_reflex_edge() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(0.0, 0.0),
        Point2::new(2.0, 0.0),
        Point2::new(2.0, 2.0),
        Point2::new(1.0, 2.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 1.0),
    ])
    .unwrap();
    topology
        .split_face_with_segment(FaceId(0), Point2::new(1.0, 1.0), Point2::new(1.0, 0.0))
        .unwrap();
    assert_eq!(topology.face_count(), 2);
    topology.validate().unwrap();
}

fn triangle_area(a: Point2, b: Point2, c: Point2) -> f32 {
    orientation(a, b, c).abs() * 0.5
}

#[test]
fn closed_loop_creates_hole_and_inner_face() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ])
    .unwrap();
    let inner = topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
                Point2::new(0.5, 0.5),
                Point2::new(-0.5, 0.5),
            ],
        )
        .unwrap();

    assert_eq!(inner, FaceId(1));
    assert_eq!(
        topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
        1
    );
    assert!(
        topology
            .face_boundary_loops(inner)
            .unwrap()
            .holes
            .is_empty()
    );
    topology.validate().unwrap();

    let triangulation = topology.triangulate_face(FaceId(0)).unwrap();
    let area: f32 = triangulation
        .triangles
        .iter()
        .map(|triangle| {
            triangle_area(
                triangulation.vertices[triangle[0] as usize],
                triangulation.vertices[triangle[1] as usize],
                triangulation.vertices[triangle[2] as usize],
            )
        })
        .sum();
    assert!((area - 15.0).abs() < 1.0e-4);
}

#[test]
fn face_can_own_multiple_disjoint_holes() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-3.0, -2.0),
        Point2::new(3.0, -2.0),
        Point2::new(3.0, 2.0),
        Point2::new(-3.0, 2.0),
    ])
    .unwrap();
    topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(-1.75, -0.5),
                Point2::new(-0.75, -0.5),
                Point2::new(-0.75, 0.5),
                Point2::new(-1.75, 0.5),
            ],
        )
        .unwrap();
    topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(0.75, -0.5),
                Point2::new(1.75, -0.5),
                Point2::new(1.75, 0.5),
                Point2::new(0.75, 0.5),
            ],
        )
        .unwrap();

    assert_eq!(
        topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
        2
    );
    assert!(topology.triangulate_face(FaceId(0)).is_ok());
    topology.validate().unwrap();
}

#[test]
fn enclosing_cut_transfers_existing_hole_to_inner_face() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-3.0, -3.0),
        Point2::new(3.0, -3.0),
        Point2::new(3.0, 3.0),
        Point2::new(-3.0, 3.0),
    ])
    .unwrap();
    topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
                Point2::new(0.5, 0.5),
                Point2::new(-0.5, 0.5),
            ],
        )
        .unwrap();
    let annulus = topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(-1.5, -1.5),
                Point2::new(1.5, -1.5),
                Point2::new(1.5, 1.5),
                Point2::new(-1.5, 1.5),
            ],
        )
        .unwrap();

    assert_eq!(
        topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
        1
    );
    assert_eq!(
        topology.face_boundary_loops(annulus).unwrap().holes.len(),
        1
    );
    assert!(topology.triangulate_face(annulus).is_ok());
    topology.validate().unwrap();
}

#[test]
fn outer_chord_split_preserves_existing_hole() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ])
    .unwrap();
    topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(-1.5, -0.4),
                Point2::new(-0.7, -0.4),
                Point2::new(-0.7, 0.4),
                Point2::new(-1.5, 0.4),
            ],
        )
        .unwrap();
    let child = topology
        .split_face_with_segment(FaceId(0), Point2::new(0.5, -2.0), Point2::new(0.5, 2.0))
        .unwrap();

    let hole_counts = [
        topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
        topology.face_boundary_loops(child).unwrap().holes.len(),
    ];
    assert_eq!(hole_counts.iter().sum::<usize>(), 1);
    topology.validate().unwrap();
}

#[test]
fn outer_to_hole_bridge_opens_annulus() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ])
    .unwrap();
    topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
                Point2::new(0.5, 0.5),
                Point2::new(-0.5, 0.5),
            ],
        )
        .unwrap();

    topology
        .bridge_boundary_components_with_polyline(
            FaceId(0),
            &[Point2::new(-2.0, 0.0), Point2::new(-0.5, 0.0)],
        )
        .unwrap();

    assert_eq!(topology.boundary_component_count(FaceId(0)).unwrap(), 1);
    assert!(
        topology
            .face_boundary_loops(FaceId(0))
            .unwrap()
            .holes
            .is_empty()
    );
    topology.validate().unwrap();
    let triangulation = topology.triangulate_face(FaceId(0)).unwrap();
    let area: f32 = triangulation
        .triangles
        .iter()
        .map(|triangle| {
            triangle_area(
                triangulation.vertices[triangle[0] as usize],
                triangulation.vertices[triangle[1] as usize],
                triangulation.vertices[triangle[2] as usize],
            )
        })
        .sum();
    assert!((area - 15.0).abs() < 1.0e-4);
}

#[test]
fn hole_to_hole_bridge_merges_inner_components() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-3.0, -2.0),
        Point2::new(3.0, -2.0),
        Point2::new(3.0, 2.0),
        Point2::new(-3.0, 2.0),
    ])
    .unwrap();
    topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(-1.75, -0.5),
                Point2::new(-0.75, -0.5),
                Point2::new(-0.75, 0.5),
                Point2::new(-1.75, 0.5),
            ],
        )
        .unwrap();
    topology
        .insert_closed_loop(
            FaceId(0),
            &[
                Point2::new(0.75, -0.5),
                Point2::new(1.75, -0.5),
                Point2::new(1.75, 0.5),
                Point2::new(0.75, 0.5),
            ],
        )
        .unwrap();

    topology
        .bridge_boundary_components_with_polyline(
            FaceId(0),
            &[Point2::new(-0.75, 0.0), Point2::new(0.75, 0.0)],
        )
        .unwrap();

    assert_eq!(topology.boundary_component_count(FaceId(0)).unwrap(), 2);
    assert_eq!(
        topology.face_boundary_loops(FaceId(0)).unwrap().holes.len(),
        1
    );
    topology.validate().unwrap();
    assert!(topology.triangulate_face(FaceId(0)).is_ok());
}

#[test]
fn boundary_bridge_rejects_same_component_transactionally() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ])
    .unwrap();
    let before = topology.clone();
    assert_eq!(
        topology.bridge_boundary_components_with_polyline(
            FaceId(0),
            &[Point2::new(-2.0, 0.0), Point2::new(2.0, 0.0)],
        ),
        Err(TopologyError::BoundaryBridgeRequiresDistinctComponents)
    );
    assert_eq!(topology, before);
}

#[test]
fn traces_path_across_existing_faces() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ])
    .unwrap();
    topology
        .split_face_with_segment(FaceId(0), Point2::new(0.0, -1.0), Point2::new(0.0, 1.0))
        .unwrap();

    let fragments = topology
        .trace_polyline_across_faces(&[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)])
        .unwrap();
    assert_eq!(fragments.len(), 2);
    assert!(approximately_equal(
        *fragments[0].last().unwrap(),
        Point2::new(0.0, 0.0)
    ));
    assert!(approximately_equal(fragments[1][0], Point2::new(0.0, 0.0)));
    assert_ne!(
        topology.face_containing_fragment(&fragments[0]).unwrap(),
        topology.face_containing_fragment(&fragments[1]).unwrap()
    );
}

#[test]
fn trace_rejects_boundary_overlap_without_mutation() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ])
    .unwrap();
    topology
        .split_face_with_segment(FaceId(0), Point2::new(0.0, -1.0), Point2::new(0.0, 1.0))
        .unwrap();
    let before = topology.clone();

    assert_eq!(
        topology.trace_polyline_across_faces(&[Point2::new(0.0, -0.75), Point2::new(0.0, 0.75),]),
        Err(TopologyError::PolylineOverlapsBoundary)
    );
    assert_eq!(topology, before);
}

#[test]
fn polyline_split_creates_half_edge_chain() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ])
    .unwrap();
    let second = topology
        .split_face_with_polyline(
            FaceId(0),
            &[
                Point2::new(-1.0, 0.0),
                Point2::new(0.0, 0.4),
                Point2::new(1.0, 0.0),
            ],
        )
        .unwrap();

    assert_eq!(second, FaceId(1));
    assert_eq!(topology.face_count(), 2);
    topology.validate().unwrap();
    assert!(topology.triangulate_face(FaceId(0)).is_ok());
    assert!(topology.triangulate_face(FaceId(1)).is_ok());
}

#[test]
fn polyline_split_is_fail_closed_when_path_leaves_face() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-1.0, -1.0),
        Point2::new(1.0, -1.0),
        Point2::new(1.0, 1.0),
        Point2::new(-1.0, 1.0),
    ])
    .unwrap();
    let before = topology.clone();
    assert_eq!(
        topology.split_face_with_polyline(
            FaceId(0),
            &[
                Point2::new(-1.0, 0.0),
                Point2::new(0.0, 1.5),
                Point2::new(1.0, 0.0),
            ],
        ),
        Err(TopologyError::PolylineLeavesFace)
    );
    assert_eq!(topology, before);
}

#[test]
fn rejects_self_intersecting_polyline_without_mutation() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(-2.0, -2.0),
        Point2::new(2.0, -2.0),
        Point2::new(2.0, 2.0),
        Point2::new(-2.0, 2.0),
    ])
    .unwrap();
    let before = topology.clone();
    assert_eq!(
        topology.split_face_with_polyline(
            FaceId(0),
            &[
                Point2::new(-2.0, 0.0),
                Point2::new(1.0, 1.0),
                Point2::new(-1.0, 1.0),
                Point2::new(2.0, 0.0),
            ],
        ),
        Err(TopologyError::PolylineSelfIntersecting)
    );
    assert_eq!(topology, before);
}

#[test]
fn rejects_chord_that_leaves_concave_face_without_mutation() {
    let mut topology = PlanarTopology::from_polygon(vec![
        Point2::new(0.0, 0.0),
        Point2::new(2.0, 0.0),
        Point2::new(2.0, 2.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 2.0),
    ])
    .unwrap();
    let before = topology.clone();
    assert_eq!(
        topology.split_face_with_segment(FaceId(0), Point2::new(0.0, 1.8), Point2::new(2.0, 1.8),),
        Err(TopologyError::SegmentLeavesFace)
    );
    assert_eq!(topology, before);
}
