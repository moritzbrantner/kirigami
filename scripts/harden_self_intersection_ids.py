from pathlib import Path

path = Path("crates/kirigami-core/src/lib.rs")
text = path.read_text()

replacements = [
(
'''    Topology(TopologyError),
    TooManyRenderVertices,
}''',
'''    Topology(TopologyError),
    TooManyRenderVertices,
    TooManyCollisionTriangles,
}'''),
(
'''            Self::TooManyRenderVertices => {
                formatter.write_str("render snapshot exceeds u32 index capacity")
            }
        }''',
'''            Self::TooManyRenderVertices => {
                formatter.write_str("render snapshot exceeds u32 index capacity")
            }
            Self::TooManyCollisionTriangles => {
                formatter.write_str("collision query exceeds u32 triangle-ID capacity")
            }
        }'''),
(
'''        let snapshot = self.render_snapshot(fold)?;
        Ok(analyze_self_intersections(&snapshot, &self.seams))
    }''',
'''        let snapshot = self.render_snapshot(fold)?;
        analyze_self_intersections(&snapshot, &self.seams)
    }'''),
(
'''fn analyze_self_intersections(snapshot: &RenderSnapshot, seams: &[Seam]) -> SelfIntersectionReport {''',
'''fn analyze_self_intersections(
    snapshot: &RenderSnapshot,
    seams: &[Seam],
) -> Result<SelfIntersectionReport, ModelError> {'''),
(
'''        bodies.push(Body::new(triangle_index as u32, Aabb::new(min, max)));''',
'''        bodies.push(Body::new(
            collision_triangle_id(triangle_index)?,
            Aabb::new(min, max),
        ));'''),
(
'''    SelfIntersectionReport {
        scope: SelfIntersectionScope::NonNeighborPanels,
        triangle_count,
        broad_phase_candidates,
        narrow_phase_tests,
        indeterminate_pairs,
        intersections,
    }
}

fn triangle_vertices''',
'''    Ok(SelfIntersectionReport {
        scope: SelfIntersectionScope::NonNeighborPanels,
        triangle_count,
        broad_phase_candidates,
        narrow_phase_tests,
        indeterminate_pairs,
        intersections,
    })
}

fn collision_triangle_id(index: usize) -> Result<u32, ModelError> {
    u32::try_from(index).map_err(|_| ModelError::TooManyCollisionTriangles)
}

fn triangle_vertices'''),
(
'''        let report = analyze_self_intersections(&snapshot, &[]);
        assert_eq!(report.triangle_count, 2);''',
'''        let report = analyze_self_intersections(&snapshot, &[]).unwrap();
        assert_eq!(report.triangle_count, 2);'''),
(
'''        let report = analyze_self_intersections(&snapshot, &[neighbor]);
        assert!(report.is_proven_clear());''',
'''        let report = analyze_self_intersections(&snapshot, &[neighbor]).unwrap();
        assert!(report.is_proven_clear());'''),
]

for old, new in replacements:
    if new in text:
        continue
    if old not in text:
        raise SystemExit(f"pattern not found: {old[:80]!r}")
    text = text.replace(old, new, 1)

marker = '''    #[test]
    fn paper_model_self_intersection_query_is_clear_at_rest() {'''
if "collision_triangle_ids_fail_closed_before_truncation" not in text:
    test = '''    #[test]
    fn collision_triangle_ids_fail_closed_before_truncation() {
        assert_eq!(
            collision_triangle_id(u32::MAX as usize + 1),
            Err(ModelError::TooManyCollisionTriangles)
        );
    }

'''
    if marker not in text:
        raise SystemExit("test marker not found")
    text = text.replace(marker, test + marker, 1)

path.write_text(text)
