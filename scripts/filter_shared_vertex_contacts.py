from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise SystemExit(f"{label}: pattern not found")
    return text.replace(old, new, 1)


topology_path = Path("crates/kirigami-core/src/topology.rs")
topology = topology_path.read_text()
topology = replace_once(
    topology,
    '''    pub fn boundary_component_count(&self, face: FaceId) -> Result<usize, TopologyError> {
        let face = self.face(face)?;
        Ok(face.holes.len() + 1)
    }

    pub fn face_polygon''',
    '''    pub fn boundary_component_count(&self, face: FaceId) -> Result<usize, TopologyError> {
        let face = self.face(face)?;
        Ok(face.holes.len() + 1)
    }

    /// Returns whether two faces intentionally meet at at least one authoritative
    /// topology vertex, including vertices on hole or bridge boundary walks.
    pub fn faces_share_vertex(
        &self,
        left: FaceId,
        right: FaceId,
    ) -> Result<bool, TopologyError> {
        let left_vertices = self.face_vertex_ids(left)?;
        let right_vertices = self.face_vertex_ids(right)?;
        Ok(left_vertices
            .iter()
            .any(|vertex| right_vertices.contains(vertex)))
    }

    pub fn face_polygon''',
    "public shared vertex query",
)

helper = '''    fn face_vertex_ids(&self, face: FaceId) -> Result<Vec<VertexId>, TopologyError> {
        let mut vertices = Vec::new();
        for boundary in self.face_boundary_half_edges(face)? {
            for edge_id in boundary {
                let vertex = self.edge(edge_id).origin;
                if !vertices.contains(&vertex) {
                    vertices.push(vertex);
                }
            }
        }
        Ok(vertices)
    }

'''
if "fn face_vertex_ids(&self" not in topology:
    marker = '''    fn face_half_edges(&self, face: FaceId) -> Result<Vec<HalfEdgeId>, TopologyError> {'''
    if marker not in topology:
        raise SystemExit("face vertex helper marker not found")
    topology = topology.replace(marker, helper + marker, 1)

topology_path.write_text(topology)

lib_path = Path("crates/kirigami-core/src/lib.rs")
lib = lib_path.read_text()
lib = replace_once(
    lib,
    '''pub enum SelfIntersectionScope {
    /// Tests only panel pairs that do not already share any cut or crease seam.
    /// This avoids reporting intended hinge/cut-boundary contact as penetration.
    NonNeighborPanels,
}''',
    '''pub enum SelfIntersectionScope {
    /// Tests only panel pairs with no intentional topological contact. Direct seam
    /// neighbors and panels sharing only a topology vertex are excluded.
    NonContactingPanels,
}''',
    "scope rename",
)

lib = replace_once(
    lib,
    '''        let snapshot = self.render_snapshot(fold)?;
        analyze_self_intersections(&snapshot, &self.seams)
    }''',
    '''        let snapshot = self.render_snapshot(fold)?;
        let intentional_contacts = self.intentional_contact_pairs()?;
        analyze_self_intersections(&snapshot, &intentional_contacts)
    }

    fn intentional_contact_pairs(&self) -> Result<HashSet<(PanelId, PanelId)>, ModelError> {
        let mut contacts: HashSet<(PanelId, PanelId)> = self
            .seams
            .iter()
            .filter(|seam| seam.panel_a != seam.panel_b)
            .map(|seam| canonical_panel_pair(seam.panel_a, seam.panel_b))
            .collect();
        for left_index in 0..self.panels.len() {
            for right_index in (left_index + 1)..self.panels.len() {
                let left = &self.panels[left_index];
                let right = &self.panels[right_index];
                if self.topology.faces_share_vertex(left.face, right.face)? {
                    contacts.insert(canonical_panel_pair(left.id, right.id));
                }
            }
        }
        Ok(contacts)
    }''',
    "intentional contacts",
)

lib = replace_once(
    lib,
    '''fn analyze_self_intersections(
    snapshot: &RenderSnapshot,
    seams: &[Seam],
) -> Result<SelfIntersectionReport, ModelError> {''',
    '''fn analyze_self_intersections(
    snapshot: &RenderSnapshot,
    intentional_contacts: &HashSet<(PanelId, PanelId)>,
) -> Result<SelfIntersectionReport, ModelError> {''',
    "analyzer signature",
)

old_neighbors = '''    let neighbors: HashSet<(PanelId, PanelId)> = seams
        .iter()
        .filter(|seam| seam.panel_a != seam.panel_b)
        .map(|seam| canonical_panel_pair(seam.panel_a, seam.panel_b))
        .collect();
    let bvh = StaticBvh::build(&bodies);'''
lib = replace_once(
    lib,
    old_neighbors,
    '''    let bvh = StaticBvh::build(&bodies);''',
    "remove seam-only neighbors",
)

lib = replace_once(
    lib,
    '''        if left_panel == right_panel
            || neighbors.contains(&canonical_panel_pair(left_panel, right_panel))
        {''',
    '''        if left_panel == right_panel
            || intentional_contacts.contains(&canonical_panel_pair(left_panel, right_panel))
        {''',
    "contact filter",
)

lib = replace_once(
    lib,
    '''        scope: SelfIntersectionScope::NonNeighborPanels,''',
    '''        scope: SelfIntersectionScope::NonContactingPanels,''',
    "scope result",
)

# Synthetic tests pass explicit contact sets now.
lib = replace_once(
    lib,
    '''        let report = analyze_self_intersections(&snapshot, &[]).unwrap();''',
    '''        let report = analyze_self_intersections(&snapshot, &HashSet::new()).unwrap();''',
    "synthetic crossing contacts",
)
lib = replace_once(
    lib,
    '''        let report = analyze_self_intersections(&snapshot, &[neighbor]).unwrap();''',
    '''        let report = analyze_self_intersections(
            &snapshot,
            &HashSet::from([canonical_panel_pair(neighbor.panel_a, neighbor.panel_b)]),
        )
        .unwrap();''',
    "neighbor contact test",
)

if "crossing_creases_shared_vertex_is_intentional_contact" not in lib:
    marker = '''    #[test]
    fn collision_triangle_ids_fail_closed_before_truncation() {'''
    test = '''    #[test]
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

'''
    if marker not in lib:
        raise SystemExit("crossing regression marker not found")
    lib = lib.replace(marker, test + marker, 1)

lib_path.write_text(lib)

roadmap_path = Path("ROADMAP.md")
roadmap = roadmap_path.read_text()
roadmap = roadmap.replace(
    "BVH-pruned, GJK-tested advisory self-intersection reports for non-neighbor panel pairs, with indeterminate pairs surfaced fail-closed.",
    "BVH-pruned, GJK-tested advisory self-intersection reports for non-contacting panel pairs, excluding direct seam neighbors and shared-topology-vertex contact; indeterminate pairs surface fail-closed.",
)
roadmap = roadmap.replace(
    "the first accepted scope intentionally excludes topological neighbor panels.",
    "the first accepted scope intentionally excludes direct seam neighbors and shared-topology-vertex contact.",
)
roadmap_path.write_text(roadmap)
