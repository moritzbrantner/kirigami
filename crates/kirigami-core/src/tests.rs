use super::*;
use crate::self_intersection::{
    analyze_self_intersections, canonical_panel_pair, collision_triangle_id,
};

fn split(kind: OperationKind) -> (PaperModel, OperationId) {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let operation = model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -0.5),
            Point2::new(0.0, 0.5),
            kind,
        )
        .unwrap();
    (model, operation)
}

#[test]
fn flat_pattern_keeps_external_boundary_and_logical_operation_paths() {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let crease = model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -0.5),
            Point2::new(0.0, 0.5),
            OperationKind::Crease,
        )
        .unwrap();
    let cut = model
        .split_across_panels_with_polyline(
            &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
            OperationKind::Cut,
        )
        .unwrap();

    let pattern = model.flat_pattern_snapshot().unwrap();
    assert_eq!(pattern.bounds.min, Point2::new(-1.0, -0.5));
    assert_eq!(pattern.bounds.max, Point2::new(1.0, 0.5));
    assert_eq!(pattern.operations.len(), 2);
    let count = |kind| {
        pattern
            .graph
            .edges
            .iter()
            .filter(|edge| edge.kind == kind)
            .count()
    };
    assert_eq!(count(FlatPatternEdgeKind::Crease), 2);
    assert_eq!(count(FlatPatternEdgeKind::Cut), 2);
    assert_eq!(count(FlatPatternEdgeKind::Boundary), 8);
    assert_eq!(pattern.graph.vertices.len(), 9);
    assert_eq!(pattern.operations[0].id, crease);
    assert_eq!(pattern.operations[0].kind, OperationKind::Crease);
    assert_eq!(pattern.operations[1].id, cut);
    assert_eq!(pattern.operations[1].kind, OperationKind::Cut);
    assert!(
        pattern
            .boundary_segments
            .iter()
            .all(|segment| segment[0].x.abs() == 1.0
                || segment[0].y.abs() == 0.5
                || segment[1].x.abs() == 1.0
                || segment[1].y.abs() == 0.5)
    );
}

#[test]
fn crease_splits_topology_but_preserves_connectivity() {
    let (model, _) = split(OperationKind::Crease);
    assert_eq!(model.panels().len(), 2);
    assert_eq!(model.topology().face_count(), 2);
    assert_eq!(model.component_count(), 1);
    model.topology().validate().unwrap();
}

#[test]
fn orthogonal_crossing_creases_pass_local_flat_foldability() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    model
        .split_across_panels_with_polyline(
            &[Point2::new(0.0, -1.0), Point2::new(0.0, 1.0)],
            OperationKind::Crease,
        )
        .unwrap();
    model
        .split_across_panels_with_polyline(
            &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
            OperationKind::Crease,
        )
        .unwrap();

    let report = model.local_flat_foldability_report();
    assert_eq!(report.checked_vertices, 1);
    assert_eq!(report.skipped_material_boundary_vertices, 0);
    assert!(report.passes_local_checks());
}

#[test]
fn t_junction_crease_fails_even_degree_condition() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, 1.0),
            OperationKind::Crease,
        )
        .unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(-1.0, 0.0),
            Point2::new(0.0, 0.0),
            OperationKind::Crease,
        )
        .unwrap();

    let report = model.local_flat_foldability_report();
    assert_eq!(report.checked_vertices, 1);
    assert!(matches!(
        report.issues.as_slice(),
        [LocalFlatFoldabilityIssue::OddCreaseDegree {
            crease_degree: 3,
            ..
        }]
    ));
}

#[test]
fn skew_crossing_creases_fail_kawasaki_condition() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    model
        .split_across_panels_with_polyline(
            &[Point2::new(0.0, -1.0), Point2::new(0.0, 1.0)],
            OperationKind::Crease,
        )
        .unwrap();
    model
        .split_across_panels_with_polyline(
            &[Point2::new(-1.0, -0.25), Point2::new(1.0, 0.25)],
            OperationKind::Crease,
        )
        .unwrap();

    let report = model.local_flat_foldability_report();
    assert_eq!(report.checked_vertices, 1);
    assert!(matches!(
        report.issues.as_slice(),
        [LocalFlatFoldabilityIssue::KawasakiViolation {
            crease_degree: 4,
            ..
        }]
    ));
}

#[test]
fn cut_splits_connectivity() {
    let (model, operation) = split(OperationKind::Cut);
    assert_eq!(model.panels().len(), 2);
    assert_eq!(model.component_count(), 2);
    assert!(matches!(
        model.render_snapshot(Some(FoldRequest {
            operation,
            angle_radians: 0.5
        })),
        Err(ModelError::CannotFoldCut(_))
    ));
}

#[test]
fn crease_fold_produces_depth_and_three_d_lab_mesh() {
    let (model, operation) = split(OperationKind::Crease);
    let fold = FoldRequest {
        operation,
        angle_radians: std::f32::consts::FRAC_PI_2,
    };
    let snapshot = model.render_snapshot(Some(fold)).unwrap();
    assert!(snapshot.vertices.iter().any(|vertex| vertex[2].abs() > 0.5));
    assert_eq!(snapshot.indices.len(), 12);
    assert_eq!(model.three_d_mesh(Some(fold)).unwrap().triangle_count(), 4);
}

#[test]
fn multiple_parallel_creases_propagate_deterministically() {
    let mut model = PaperModel::rectangle(3.0, 1.0).unwrap();
    let first = model
        .split_across_panels_with_polyline(
            &[Point2::new(-0.5, -0.5), Point2::new(-0.5, 0.5)],
            OperationKind::Crease,
        )
        .unwrap();
    let second = model
        .split_across_panels_with_polyline(
            &[Point2::new(0.5, -0.5), Point2::new(0.5, 0.5)],
            OperationKind::Crease,
        )
        .unwrap();

    let folds = [
        FoldRequest {
            operation: first,
            angle_radians: 0.7,
        },
        FoldRequest {
            operation: second,
            angle_radians: -0.45,
        },
    ];
    let forward = model.render_snapshot_with_folds(&folds).unwrap();
    let reverse = model
        .render_snapshot_with_folds(&[folds[1], folds[0]])
        .unwrap();

    assert_eq!(forward, reverse);
    assert!(forward.vertices.iter().any(|vertex| vertex[2].abs() > 0.25));
    assert_eq!(
        model
            .three_d_mesh_with_folds(&folds)
            .unwrap()
            .triangle_count(),
        6
    );
}

#[test]
fn crossing_crease_state_fails_closed_when_axis_becomes_kinked() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    let vertical = model
        .split_across_panels_with_polyline(
            &[Point2::new(0.0, -1.0), Point2::new(0.0, 1.0)],
            OperationKind::Crease,
        )
        .unwrap();
    let horizontal = model
        .split_across_panels_with_polyline(
            &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
            OperationKind::Crease,
        )
        .unwrap();

    assert!(matches!(
        model.render_snapshot_with_folds(&[
            FoldRequest {
                operation: vertical,
                angle_radians: 0.6,
            },
            FoldRequest {
                operation: horizontal,
                angle_radians: 0.4,
            },
        ]),
        Err(ModelError::FoldConstraintConflict(operation)) if operation == horizontal
    ));
}

#[test]
fn duplicate_fold_requests_fail_closed() {
    let (model, operation) = split(OperationKind::Crease);
    assert!(matches!(
        model.render_snapshot_with_folds(&[
            FoldRequest {
                operation,
                angle_radians: 0.4,
            },
            FoldRequest {
                operation,
                angle_radians: 0.5,
            },
        ]),
        Err(ModelError::DuplicateFoldOperation(duplicate)) if duplicate == operation
    ));
}

#[test]
fn split_rejects_non_boundary_endpoints_without_mutation() {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let before = model.clone();
    assert_eq!(
        model.split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 0.5),
            OperationKind::Cut,
        ),
        Err(ModelError::SegmentEndpointOffBoundary)
    );
    assert_eq!(model, before);
}

#[test]
fn later_split_preserves_existing_crease_adjacency() {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let crease = model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -0.5),
            Point2::new(0.0, 0.5),
            OperationKind::Crease,
        )
        .unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(-1.0, 0.0),
            Point2::new(0.0, 0.0),
            OperationKind::Cut,
        )
        .unwrap();

    let crease_segments: Vec<&Seam> = model
        .seams()
        .iter()
        .filter(|seam| seam.operation == crease)
        .collect();
    assert_eq!(crease_segments.len(), 2);
    assert_eq!(model.component_count(), 1);
    assert_eq!(model.topology().face_count(), 3);
    model.topology().validate().unwrap();
    assert!(
        crease_segments
            .iter()
            .any(|seam| seam.panel_a == PanelId(0))
    );
    assert!(
        crease_segments
            .iter()
            .any(|seam| seam.panel_a == PanelId(2))
    );
}

#[test]
fn folding_subdivided_crease_excludes_all_of_its_segments() {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let crease = model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -0.5),
            Point2::new(0.0, 0.5),
            OperationKind::Crease,
        )
        .unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(-1.0, 0.0),
            Point2::new(0.0, 0.0),
            OperationKind::Cut,
        )
        .unwrap();

    let snapshot = model
        .render_snapshot(Some(FoldRequest {
            operation: crease,
            angle_radians: std::f32::consts::FRAC_PI_2,
        }))
        .unwrap();
    assert!(snapshot.vertices.iter().any(|vertex| vertex[2].abs() > 0.5));
}

#[test]
fn cut_across_existing_crease_is_one_operation() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    let first_crease = model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, 1.0),
            OperationKind::Crease,
        )
        .unwrap();
    let cut = model
        .split_across_panels_with_polyline(
            &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
            OperationKind::Cut,
        )
        .unwrap();

    assert_eq!(model.panels().len(), 4);
    assert_eq!(model.topology().face_count(), 4);
    assert_eq!(model.component_count(), 2);
    assert_eq!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == cut)
            .count(),
        2
    );
    assert_eq!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == first_crease)
            .count(),
        2
    );
    assert_eq!(model.operations().last().unwrap().path.len(), 2);
    model.topology().validate().unwrap();
}

#[test]
fn bent_cut_across_existing_crease_preserves_one_operation() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, 1.0),
            OperationKind::Crease,
        )
        .unwrap();
    let path = [
        Point2::new(-1.0, -0.5),
        Point2::new(-0.25, 0.25),
        Point2::new(0.75, 0.25),
        Point2::new(1.0, -0.5),
    ];
    let cut = model
        .split_across_panels_with_polyline(&path, OperationKind::Cut)
        .unwrap();

    assert_eq!(model.panels().len(), 4);
    assert_eq!(model.component_count(), 2);
    assert_eq!(model.operations().last().unwrap().path, path);
    assert_eq!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == cut)
            .count(),
        4
    );
    model.topology().validate().unwrap();
}

#[test]
fn straight_crease_can_cross_existing_crease_with_one_fold_side() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, 1.0),
            OperationKind::Crease,
        )
        .unwrap();
    let horizontal = model
        .split_across_panels_with_polyline(
            &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
            OperationKind::Crease,
        )
        .unwrap();

    assert_eq!(model.component_count(), 1);
    assert_eq!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == horizontal)
            .count(),
        2
    );
    let resolved = model
        .resolve_folds(&[FoldRequest {
            operation: horizontal,
            angle_radians: std::f32::consts::FRAC_PI_2,
        }])
        .unwrap();
    assert_eq!(resolved[0].moving_panels.len(), 2);
    let snapshot = model
        .render_snapshot(Some(FoldRequest {
            operation: horizontal,
            angle_radians: std::f32::consts::FRAC_PI_2,
        }))
        .unwrap();
    assert!(snapshot.vertices.iter().any(|vertex| vertex[2].abs() > 0.5));
}

#[test]
fn multi_face_path_failure_is_transactional() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, 1.0),
            OperationKind::Crease,
        )
        .unwrap();
    let before = model.clone();
    assert_eq!(
        model.split_across_panels_with_polyline(
            &[
                Point2::new(-1.0, 0.0),
                Point2::new(0.0, 0.0),
                Point2::new(0.0, 0.75),
            ],
            OperationKind::Cut,
        ),
        Err(ModelError::SegmentDoesNotSplitPanel)
    );
    assert_eq!(model, before);
}

#[test]
fn closed_cut_creates_detached_panel_and_hole() {
    let mut model = PaperModel::rectangle(4.0, 4.0).unwrap();
    let cut = model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
                Point2::new(0.5, 0.5),
                Point2::new(-0.5, 0.5),
            ],
        )
        .unwrap();

    assert_eq!(model.panels().len(), 2);
    assert_eq!(model.component_count(), 2);
    assert_eq!(
        model.operations().last().unwrap().path.first(),
        model.operations().last().unwrap().path.last()
    );
    assert_eq!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == cut)
            .count(),
        4
    );
    assert_eq!(
        model
            .topology()
            .face_boundary_loops(FaceId(0))
            .unwrap()
            .holes
            .len(),
        1
    );
    let snapshot = model.render_snapshot(None).unwrap();
    assert_eq!(snapshot.panel_count, 2);
    assert_eq!(snapshot.component_count, 2);
}

#[test]
fn bvh_self_intersection_finds_non_neighbor_crossing() {
    let snapshot = RenderSnapshot {
        vertices: vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.25, 0.25, -1.0],
            [0.25, 0.25, 1.0],
            [0.75, 0.25, 0.0],
        ],
        indices: vec![0, 1, 2, 3, 4, 5],
        triangle_panels: vec![PanelId(0), PanelId(2)],
        seams: Vec::new(),
        panel_count: 2,
        component_count: 2,
    };
    let report = analyze_self_intersections(&snapshot, &HashSet::new()).unwrap();
    assert_eq!(report.triangle_count, 2);
    assert_eq!(report.narrow_phase_tests, 1);
    assert_eq!(report.intersections.len(), 1);
    assert!(report.indeterminate_pairs.is_empty());
    assert!(!report.is_proven_clear());
}

#[test]
fn contact_filter_allows_intended_neighbor_boundary_contact() {
    let snapshot = RenderSnapshot {
        vertices: vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, -1.0, 0.0],
        ],
        indices: vec![0, 1, 2, 3, 4, 5],
        triangle_panels: vec![PanelId(0), PanelId(1)],
        seams: Vec::new(),
        panel_count: 2,
        component_count: 1,
    };
    let neighbor = Seam {
        id: SeamId(0),
        operation: OperationId(0),
        kind: OperationKind::Crease,
        start: Point2::new(0.0, 0.0),
        end: Point2::new(1.0, 0.0),
        panel_a: PanelId(0),
        panel_b: PanelId(1),
    };
    let report = analyze_self_intersections(
        &snapshot,
        &HashSet::from([canonical_panel_pair(neighbor.panel_a, neighbor.panel_b)]),
    )
    .unwrap();
    assert_eq!(
        report.scope,
        SelfIntersectionScope::AllPanelPairsContactAware
    );
    assert!(report.is_proven_clear());
    assert!(report.narrow_phase_tests >= 1);
}

#[test]
fn adjacent_fold_contact_is_clear_but_full_overlap_is_reported() {
    let mut model = PaperModel::rectangle(4.0, 2.0).unwrap();
    let operation = model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, 1.0),
            OperationKind::Crease,
        )
        .unwrap();

    let hinge_contact = model
        .self_intersection_report(Some(FoldRequest {
            operation,
            angle_radians: std::f32::consts::FRAC_PI_2,
        }))
        .unwrap();
    assert!(hinge_contact.is_proven_clear());

    let overlap = model
        .self_intersection_report(Some(FoldRequest {
            operation,
            angle_radians: std::f32::consts::PI,
        }))
        .unwrap();
    assert!(!overlap.is_proven_clear());
    assert!(!overlap.intersections.is_empty());
}

#[test]
fn crossing_creases_shared_vertex_is_intentional_contact() {
    let mut model = PaperModel::rectangle(2.0, 2.0).unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, 1.0),
            OperationKind::Crease,
        )
        .unwrap();
    model
        .split_across_panels_with_polyline(
            &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
            OperationKind::Crease,
        )
        .unwrap();

    let contacts = model.intentional_contact_pairs().unwrap();
    assert!(contacts.contains(&canonical_panel_pair(PanelId(0), PanelId(3))));
    let report = model.self_intersection_report(None).unwrap();
    assert!(report.is_proven_clear());
    assert!(report.intersections.is_empty());
    assert!(report.indeterminate_pairs.is_empty());
}

#[test]
fn collision_triangle_ids_fail_closed_before_truncation() {
    assert_eq!(
        collision_triangle_id(u32::MAX as usize + 1),
        Err(ModelError::TooManyCollisionTriangles)
    );
}

#[test]
fn paper_model_self_intersection_query_is_clear_at_rest() {
    let mut model = PaperModel::rectangle(4.0, 2.0).unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -1.0),
            Point2::new(0.0, 1.0),
            OperationKind::Crease,
        )
        .unwrap();
    let report = model.self_intersection_report(None).unwrap();
    assert!(report.is_proven_clear());
    assert!(report.intersections.is_empty());
}

#[test]
fn boundary_bridge_cut_opens_annulus_without_new_panel() {
    let mut model = PaperModel::rectangle(4.0, 4.0).unwrap();
    model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
                Point2::new(0.5, 0.5),
                Point2::new(-0.5, 0.5),
            ],
        )
        .unwrap();
    let panel_count = model.panels().len();
    let bridge = model
        .cut_boundary_bridge(
            PanelId(0),
            &[Point2::new(-2.0, 0.0), Point2::new(-0.5, 0.0)],
        )
        .unwrap();

    assert_eq!(model.panels().len(), panel_count);
    assert_eq!(model.component_count(), 2);
    assert_eq!(
        model
            .topology()
            .boundary_component_count(FaceId(0))
            .unwrap(),
        1
    );
    let bridge_segments: Vec<&Seam> = model
        .seams()
        .iter()
        .filter(|seam| seam.operation == bridge)
        .collect();
    assert_eq!(bridge_segments.len(), 1);
    assert!(
        bridge_segments
            .iter()
            .all(|seam| seam.panel_a == PanelId(0) && seam.panel_b == PanelId(0))
    );
    assert!(model.render_snapshot(None).is_ok());
}

#[test]
fn boundary_bridge_can_merge_two_holes_without_new_panel() {
    let mut model = PaperModel::rectangle(6.0, 4.0).unwrap();
    model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(-1.75, -0.5),
                Point2::new(-0.75, -0.5),
                Point2::new(-0.75, 0.5),
                Point2::new(-1.75, 0.5),
            ],
        )
        .unwrap();
    model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(0.75, -0.5),
                Point2::new(1.75, -0.5),
                Point2::new(1.75, 0.5),
                Point2::new(0.75, 0.5),
            ],
        )
        .unwrap();
    let before = model.panels().len();
    model
        .cut_boundary_bridge(
            PanelId(0),
            &[Point2::new(-0.75, 0.0), Point2::new(0.75, 0.0)],
        )
        .unwrap();

    assert_eq!(model.panels().len(), before);
    assert_eq!(
        model
            .topology()
            .boundary_component_count(FaceId(0))
            .unwrap(),
        2
    );
    assert_eq!(model.component_count(), 3);
    assert!(model.render_snapshot(None).is_ok());
}

#[test]
fn second_disjoint_closed_cut_adds_second_hole() {
    let mut model = PaperModel::rectangle(6.0, 4.0).unwrap();
    model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(-2.0, -0.5),
                Point2::new(-1.0, -0.5),
                Point2::new(-1.0, 0.5),
                Point2::new(-2.0, 0.5),
            ],
        )
        .unwrap();
    model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(1.0, -0.5),
                Point2::new(2.0, -0.5),
                Point2::new(2.0, 0.5),
                Point2::new(1.0, 0.5),
            ],
        )
        .unwrap();

    assert_eq!(model.panels().len(), 3);
    assert_eq!(model.component_count(), 3);
    assert_eq!(
        model
            .topology()
            .face_boundary_loops(FaceId(0))
            .unwrap()
            .holes
            .len(),
        2
    );
    assert!(model.render_snapshot(None).is_ok());
}

#[test]
fn enclosing_closed_cut_moves_existing_cut_boundary_to_new_panel() {
    let mut model = PaperModel::rectangle(6.0, 6.0).unwrap();
    let inner_cut = model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
                Point2::new(0.5, 0.5),
                Point2::new(-0.5, 0.5),
            ],
        )
        .unwrap();
    model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(-1.5, -1.5),
                Point2::new(1.5, -1.5),
                Point2::new(1.5, 1.5),
                Point2::new(-1.5, 1.5),
            ],
        )
        .unwrap();

    let annulus_panel = PanelId(2);
    assert!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == inner_cut)
            .all(|seam| seam.panel_a == annulus_panel || seam.panel_b == annulus_panel)
    );
    assert_eq!(
        model
            .topology()
            .face_boundary_loops(
                model
                    .panels()
                    .iter()
                    .find(|panel| panel.id == annulus_panel)
                    .unwrap()
                    .face
            )
            .unwrap()
            .holes
            .len(),
        1
    );
    model.topology().validate().unwrap();
}

#[test]
fn open_cut_after_closed_cut_preserves_hole_seam_provenance() {
    let mut model = PaperModel::rectangle(6.0, 4.0).unwrap();
    let closed = model
        .cut_closed_path(
            PanelId(0),
            &[
                Point2::new(-2.0, -0.5),
                Point2::new(-1.0, -0.5),
                Point2::new(-1.0, 0.5),
                Point2::new(-2.0, 0.5),
            ],
        )
        .unwrap();
    model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.5, -2.0),
            Point2::new(0.5, 2.0),
            OperationKind::Cut,
        )
        .unwrap();

    assert_eq!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == closed)
            .count(),
        4
    );
    assert!(model.render_snapshot(None).is_ok());
    model.topology().validate().unwrap();
}

#[test]
fn bent_cut_creates_segmented_operation_and_disconnects() {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let operation = model
        .split_panel_with_polyline(
            PanelId(0),
            &[
                Point2::new(-1.0, 0.0),
                Point2::new(0.0, 0.25),
                Point2::new(1.0, 0.0),
            ],
            OperationKind::Cut,
        )
        .unwrap();

    assert_eq!(model.panels().len(), 2);
    assert_eq!(model.component_count(), 2);
    assert_eq!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == operation)
            .count(),
        2
    );
    assert_eq!(model.operations()[0].path.len(), 3);
    model.topology().validate().unwrap();
}

#[test]
fn bent_crease_is_deferred_without_mutation() {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let before = model.clone();
    assert_eq!(
        model.split_panel_with_polyline(
            PanelId(0),
            &[
                Point2::new(-1.0, 0.0),
                Point2::new(0.0, 0.25),
                Point2::new(1.0, 0.0),
            ],
            OperationKind::Crease,
        ),
        Err(ModelError::BentCreaseRequiresConstraintSolver)
    );
    assert_eq!(model, before);
}

#[test]
fn collinear_segmented_crease_still_uses_rigid_fold_axis() {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let crease = model
        .split_panel_with_polyline(
            PanelId(0),
            &[
                Point2::new(0.0, -0.5),
                Point2::new(0.0, 0.0),
                Point2::new(0.0, 0.5),
            ],
            OperationKind::Crease,
        )
        .unwrap();
    let snapshot = model
        .render_snapshot(Some(FoldRequest {
            operation: crease,
            angle_radians: std::f32::consts::FRAC_PI_2,
        }))
        .unwrap();
    assert!(snapshot.vertices.iter().any(|vertex| vertex[2].abs() > 0.5));
    assert_eq!(
        model
            .seams()
            .iter()
            .filter(|seam| seam.operation == crease)
            .count(),
        2
    );
}

#[test]
fn downstream_crease_moves_with_folded_component() {
    let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
    let first = model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -0.5),
            Point2::new(0.0, 0.5),
            OperationKind::Crease,
        )
        .unwrap();
    let second = model
        .split_panel_with_segment(
            PanelId(1),
            Point2::new(0.5, -0.5),
            Point2::new(0.5, 0.5),
            OperationKind::Crease,
        )
        .unwrap();

    let snapshot = model
        .render_snapshot(Some(FoldRequest {
            operation: first,
            angle_radians: std::f32::consts::FRAC_PI_2,
        }))
        .unwrap();
    let downstream = snapshot
        .seams
        .iter()
        .find(|seam| seam.operation == second)
        .unwrap();
    assert!(downstream.start[2].abs() > 0.4);
    assert!(downstream.end[2].abs() > 0.4);
}
