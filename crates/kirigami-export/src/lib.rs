//! Deterministic file export adapters for authoritative Kirigami flat patterns.
//!
//! Paper semantics remain in `kirigami-core`. This crate only maps a
//! `FlatPatternSnapshot` into portable print/vector formats.

use kirigami_core::{FlatPatternSnapshot, OperationKind, Point2};
use serde::Serialize;
use std::fmt;

const POINTS_PER_MM: f32 = 72.0 / 25.4;
const EPSILON: f32 = 1.0e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PdfExportOptions {
    pub page_width_mm: f32,
    pub page_height_mm: f32,
    pub margin_mm: f32,
    /// Physical width of the material boundary on the printed page.
    pub template_width_mm: f32,
}

impl PdfExportOptions {
    pub const fn a4(template_width_mm: f32) -> Self {
        Self {
            page_width_mm: 210.0,
            page_height_mm: 297.0,
            margin_mm: 10.0,
            template_width_mm,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    InvalidPageSize,
    InvalidMargin,
    InvalidTemplateWidth,
    DegeneratePattern,
    TemplateDoesNotFit { width_mm: f32, height_mm: f32 },
    Serialization(String),
}

impl fmt::Display for ExportError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPageSize => {
                formatter.write_str("page dimensions must be finite and positive")
            }
            Self::InvalidMargin => {
                formatter.write_str("page margin must be finite and non-negative")
            }
            Self::InvalidTemplateWidth => {
                formatter.write_str("template width must be finite and positive")
            }
            Self::DegeneratePattern => formatter.write_str("flat pattern has zero width or height"),
            Self::TemplateDoesNotFit {
                width_mm,
                height_mm,
            } => write!(
                formatter,
                "template size {width_mm:.1} x {height_mm:.1} mm does not fit the printable page area"
            ),
            Self::Serialization(error) => write!(formatter, "could not serialize export: {error}"),
        }
    }
}

impl std::error::Error for ExportError {}

#[derive(Debug, Clone, Copy)]
struct Layout {
    source_min: Point2,
    scale_mm_per_unit: f32,
    offset_x_mm: f32,
    offset_y_mm: f32,
}

pub fn export_pdf(
    pattern: &FlatPatternSnapshot,
    options: PdfExportOptions,
) -> Result<Vec<u8>, ExportError> {
    let layout = layout(pattern, options)?;
    let mut content = String::from("q\n0 G\n1 J\n1 j\n");

    content.push_str(&format!("{:.3} w\n[] 0 d\n", mm_to_points(0.35)));
    for segment in &pattern.boundary_segments {
        push_pdf_segment(&mut content, &layout, segment[0], segment[1]);
    }

    content.push_str(&format!("{:.3} w\n[] 0 d\n", mm_to_points(0.25)));
    for operation in pattern
        .operations
        .iter()
        .filter(|operation| operation.kind == OperationKind::Cut)
    {
        push_pdf_path(&mut content, &layout, &operation.path);
    }

    content.push_str(&format!(
        "{:.3} w\n[{:.3} {:.3}] 0 d\n",
        mm_to_points(0.2),
        mm_to_points(2.0),
        mm_to_points(1.5)
    ));
    for operation in pattern
        .operations
        .iter()
        .filter(|operation| operation.kind == OperationKind::Crease)
    {
        push_pdf_path(&mut content, &layout, &operation.path);
    }
    content.push_str("Q\n");

    Ok(build_pdf(
        options.page_width_mm * POINTS_PER_MM,
        options.page_height_mm * POINTS_PER_MM,
        content.as_bytes(),
    ))
}

pub fn export_svg(
    pattern: &FlatPatternSnapshot,
    template_width_mm: f32,
) -> Result<String, ExportError> {
    if !template_width_mm.is_finite() || template_width_mm <= 0.0 {
        return Err(ExportError::InvalidTemplateWidth);
    }
    let source_width = pattern.bounds.max.x - pattern.bounds.min.x;
    let source_height = pattern.bounds.max.y - pattern.bounds.min.y;
    if source_width <= EPSILON || source_height <= EPSILON {
        return Err(ExportError::DegeneratePattern);
    }

    let scale = template_width_mm / source_width;
    let height_mm = source_height * scale;
    let mut svg = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{template_width_mm:.3}mm\" height=\"{height_mm:.3}mm\" viewBox=\"0 0 {template_width_mm:.3} {height_mm:.3}\">\n"
    );
    svg.push_str(
        "  <g fill=\"none\" stroke=\"#000\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
    );
    svg.push_str("    <g stroke-width=\"0.35\">\n");
    for segment in &pattern.boundary_segments {
        push_svg_segment(&mut svg, pattern, scale, height_mm, segment[0], segment[1]);
    }
    svg.push_str("    </g>\n    <g stroke-width=\"0.25\">\n");
    for operation in pattern
        .operations
        .iter()
        .filter(|operation| operation.kind == OperationKind::Cut)
    {
        push_svg_path(&mut svg, pattern, scale, height_mm, &operation.path, false);
    }
    svg.push_str("    </g>\n    <g stroke-width=\"0.20\" stroke-dasharray=\"2 1.5\">\n");
    for operation in pattern
        .operations
        .iter()
        .filter(|operation| operation.kind == OperationKind::Crease)
    {
        push_svg_path(&mut svg, pattern, scale, height_mm, &operation.path, true);
    }
    svg.push_str("    </g>\n  </g>\n</svg>\n");
    Ok(svg)
}

#[derive(Debug, Serialize)]
struct FoldDocument {
    file_spec: f32,
    file_creator: &'static str,
    file_classes: [&'static str; 1],
    frame_classes: [&'static str; 1],
    frame_attributes: Vec<&'static str>,
    frame_unit: &'static str,
    vertices_coords: Vec<[f32; 2]>,
    edges_vertices: Vec<[u32; 2]>,
    edges_assignment: Vec<&'static str>,
}

/// Exports a FOLD 1.2 crease-pattern graph at an explicit physical width.
///
/// External material boundaries use `B`, cuts use the FOLD 1.2 `C`
/// assignment, and creases remain `U` until Kirigami owns an explicit
/// mountain/valley assignment. Geometrically subdivided seam segments are
/// used so crossing operations retain their intersection vertices.
pub fn export_fold(
    pattern: &FlatPatternSnapshot,
    template_width_mm: f32,
) -> Result<String, ExportError> {
    if !template_width_mm.is_finite() || template_width_mm <= 0.0 {
        return Err(ExportError::InvalidTemplateWidth);
    }
    let source_width = pattern.bounds.max.x - pattern.bounds.min.x;
    let source_height = pattern.bounds.max.y - pattern.bounds.min.y;
    if source_width <= EPSILON || source_height <= EPSILON {
        return Err(ExportError::DegeneratePattern);
    }

    let mut graph = FoldGraphBuilder::new(
        pattern.bounds.min,
        template_width_mm / source_width,
    );
    for segment in &pattern.boundary_segments {
        graph.push_edge(segment[0], segment[1], "B")?;
    }
    for segment in &pattern.segments {
        let assignment = match segment.kind {
            OperationKind::Cut => "C",
            OperationKind::Crease => "U",
        };
        graph.push_edge(segment.start, segment.end, assignment)?;
    }

    let mut frame_attributes = vec!["2D"];
    if pattern
        .segments
        .iter()
        .any(|segment| segment.kind == OperationKind::Cut)
    {
        frame_attributes.push("cuts");
    }

    serde_json::to_string_pretty(&FoldDocument {
        file_spec: 1.2,
        file_creator: "kirigami",
        file_classes: ["singleModel"],
        frame_classes: ["creasePattern"],
        frame_attributes,
        frame_unit: "mm",
        vertices_coords: graph.vertices,
        edges_vertices: graph.edges,
        edges_assignment: graph.assignments,
    })
    .map(|json| format!("{json}\n"))
    .map_err(|error| ExportError::Serialization(error.to_string()))
}

struct FoldGraphBuilder {
    source_min: Point2,
    scale: f32,
    vertices: Vec<[f32; 2]>,
    edges: Vec<[u32; 2]>,
    assignments: Vec<&'static str>,
}

impl FoldGraphBuilder {
    fn new(source_min: Point2, scale: f32) -> Self {
        Self {
            source_min,
            scale,
            vertices: Vec::new(),
            edges: Vec::new(),
            assignments: Vec::new(),
        }
    }

    fn push_edge(
        &mut self,
        start: Point2,
        end: Point2,
        assignment: &'static str,
    ) -> Result<(), ExportError> {
        let start = self.point(start);
        let end = self.point(end);
        let start = self.vertex_id(start)?;
        let end = self.vertex_id(end)?;
        self.edges.push([start, end]);
        self.assignments.push(assignment);
        Ok(())
    }

    fn point(&self, point: Point2) -> [f32; 2] {
        [
            (point.x - self.source_min.x) * self.scale,
            (point.y - self.source_min.y) * self.scale,
        ]
    }

    fn vertex_id(&mut self, point: [f32; 2]) -> Result<u32, ExportError> {
        if let Some(index) = self.vertices.iter().position(|candidate| {
            (candidate[0] - point[0]).abs() <= EPSILON
                && (candidate[1] - point[1]).abs() <= EPSILON
        }) {
            return u32::try_from(index)
                .map_err(|error| ExportError::Serialization(error.to_string()));
        }

        let index = u32::try_from(self.vertices.len())
            .map_err(|error| ExportError::Serialization(error.to_string()))?;
        self.vertices.push(point);
        Ok(index)
    }
}

fn layout(pattern: &FlatPatternSnapshot, options: PdfExportOptions) -> Result<Layout, ExportError> {
    if !options.page_width_mm.is_finite()
        || !options.page_height_mm.is_finite()
        || options.page_width_mm <= 0.0
        || options.page_height_mm <= 0.0
    {
        return Err(ExportError::InvalidPageSize);
    }
    if !options.margin_mm.is_finite() || options.margin_mm < 0.0 {
        return Err(ExportError::InvalidMargin);
    }
    if !options.template_width_mm.is_finite() || options.template_width_mm <= 0.0 {
        return Err(ExportError::InvalidTemplateWidth);
    }

    let source_width = pattern.bounds.max.x - pattern.bounds.min.x;
    let source_height = pattern.bounds.max.y - pattern.bounds.min.y;
    if source_width <= EPSILON || source_height <= EPSILON {
        return Err(ExportError::DegeneratePattern);
    }

    let scale_mm_per_unit = options.template_width_mm / source_width;
    let width_mm = options.template_width_mm;
    let height_mm = source_height * scale_mm_per_unit;
    let printable_width = options.page_width_mm - options.margin_mm * 2.0;
    let printable_height = options.page_height_mm - options.margin_mm * 2.0;
    if printable_width <= 0.0
        || printable_height <= 0.0
        || width_mm > printable_width + EPSILON
        || height_mm > printable_height + EPSILON
    {
        return Err(ExportError::TemplateDoesNotFit {
            width_mm,
            height_mm,
        });
    }

    Ok(Layout {
        source_min: pattern.bounds.min,
        scale_mm_per_unit,
        offset_x_mm: (options.page_width_mm - width_mm) * 0.5,
        offset_y_mm: (options.page_height_mm - height_mm) * 0.5,
    })
}

fn push_pdf_segment(content: &mut String, layout: &Layout, start: Point2, end: Point2) {
    let start = pdf_point(layout, start);
    let end = pdf_point(layout, end);
    content.push_str(&format!(
        "{:.3} {:.3} m\n{:.3} {:.3} l\nS\n",
        start.0, start.1, end.0, end.1
    ));
}

fn push_pdf_path(content: &mut String, layout: &Layout, path: &[Point2]) {
    let Some(first) = path.first().copied() else {
        return;
    };
    let first = pdf_point(layout, first);
    content.push_str(&format!("{:.3} {:.3} m\n", first.0, first.1));
    for point in &path[1..] {
        let point = pdf_point(layout, *point);
        content.push_str(&format!("{:.3} {:.3} l\n", point.0, point.1));
    }
    content.push_str("S\n");
}

fn pdf_point(layout: &Layout, point: Point2) -> (f32, f32) {
    let x_mm = layout.offset_x_mm + (point.x - layout.source_min.x) * layout.scale_mm_per_unit;
    let y_mm = layout.offset_y_mm + (point.y - layout.source_min.y) * layout.scale_mm_per_unit;
    (mm_to_points(x_mm), mm_to_points(y_mm))
}

fn push_svg_segment(
    svg: &mut String,
    pattern: &FlatPatternSnapshot,
    scale: f32,
    height_mm: f32,
    start: Point2,
    end: Point2,
) {
    let start = svg_point(pattern, scale, height_mm, start);
    let end = svg_point(pattern, scale, height_mm, end);
    svg.push_str(&format!(
        "      <path d=\"M {:.3} {:.3} L {:.3} {:.3}\"/>\n",
        start.0, start.1, end.0, end.1
    ));
}

fn push_svg_path(
    svg: &mut String,
    pattern: &FlatPatternSnapshot,
    scale: f32,
    height_mm: f32,
    path: &[Point2],
    _dashed: bool,
) {
    let Some(first) = path.first().copied() else {
        return;
    };
    let first = svg_point(pattern, scale, height_mm, first);
    svg.push_str(&format!("      <path d=\"M {:.3} {:.3}", first.0, first.1));
    for point in &path[1..] {
        let point = svg_point(pattern, scale, height_mm, *point);
        svg.push_str(&format!(" L {:.3} {:.3}", point.0, point.1));
    }
    svg.push_str("\"/>\n");
}

fn svg_point(
    pattern: &FlatPatternSnapshot,
    scale: f32,
    height_mm: f32,
    point: Point2,
) -> (f32, f32) {
    let x = (point.x - pattern.bounds.min.x) * scale;
    let y = height_mm - (point.y - pattern.bounds.min.y) * scale;
    (x, y)
}

fn mm_to_points(value: f32) -> f32 {
    value * POINTS_PER_MM
}

fn build_pdf(page_width_points: f32, page_height_points: f32, stream: &[u8]) -> Vec<u8> {
    let mut pdf = Vec::new();
    pdf.extend_from_slice(b"%PDF-1.4\n%\xE2\xE3\xCF\xD3\n");
    let mut offsets = Vec::with_capacity(4);

    push_object(
        &mut pdf,
        &mut offsets,
        1,
        b"<< /Type /Catalog /Pages 2 0 R >>",
    );
    push_object(
        &mut pdf,
        &mut offsets,
        2,
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
    );
    let page = format!(
        "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {page_width_points:.3} {page_height_points:.3}] /Resources << >> /Contents 4 0 R >>"
    );
    push_object(&mut pdf, &mut offsets, 3, page.as_bytes());

    let mut stream_object = format!("<< /Length {} >>\nstream\n", stream.len()).into_bytes();
    stream_object.extend_from_slice(stream);
    stream_object.extend_from_slice(b"endstream");
    push_object(&mut pdf, &mut offsets, 4, &stream_object);

    let xref_offset = pdf.len();
    pdf.extend_from_slice(b"xref\n0 5\n0000000000 65535 f \n");
    for offset in offsets {
        pdf.extend_from_slice(format!("{offset:010} 00000 n \n").as_bytes());
    }
    pdf.extend_from_slice(
        format!("trailer\n<< /Size 5 /Root 1 0 R >>\nstartxref\n{xref_offset}\n%%EOF\n").as_bytes(),
    );
    pdf
}

fn push_object(pdf: &mut Vec<u8>, offsets: &mut Vec<usize>, id: usize, body: &[u8]) {
    offsets.push(pdf.len());
    pdf.extend_from_slice(format!("{id} 0 obj\n").as_bytes());
    pdf.extend_from_slice(body);
    pdf.extend_from_slice(b"\nendobj\n");
}

#[cfg(test)]
mod tests {
    use super::*;
    use kirigami_core::{OperationKind, PanelId, PaperModel, Point2};

    fn sample_pattern() -> FlatPatternSnapshot {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -0.5),
                Point2::new(0.0, 0.5),
                OperationKind::Crease,
            )
            .unwrap();
        model
            .split_across_panels_with_polyline(
                &[Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
                OperationKind::Cut,
            )
            .unwrap();
        model.flat_pattern_snapshot().unwrap()
    }

    #[test]
    fn pdf_is_single_page_vector_output_with_cut_and_crease_styles() {
        let pdf = export_pdf(&sample_pattern(), PdfExportOptions::a4(180.0)).unwrap();
        assert!(pdf.starts_with(b"%PDF-1.4"));
        let text = String::from_utf8_lossy(&pdf);
        assert!(text.contains("/Type /Page"));
        assert!(text.contains("[] 0 d"));
        assert!(text.contains("[5.669 4.252] 0 d"));
        assert!(text.ends_with("%%EOF\n"));
    }

    #[test]
    fn svg_preserves_exact_requested_physical_width() {
        let svg = export_svg(&sample_pattern(), 180.0).unwrap();
        assert!(svg.contains("width=\"180.000mm\""));
        assert!(svg.contains("height=\"90.000mm\""));
        assert!(svg.contains("stroke-dasharray=\"2 1.5\""));
    }

    #[test]
    fn pdf_rejects_a_template_that_exceeds_the_printable_page() {
        assert!(matches!(
            export_pdf(&sample_pattern(), PdfExportOptions::a4(250.0)),
            Err(ExportError::TemplateDoesNotFit { .. })
        ));
    }
    #[test]
    fn fold_export_preserves_crossing_vertices_and_unassigned_creases() {
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

        let fold = export_fold(&model.flat_pattern_snapshot().unwrap(), 180.0).unwrap();
        let document: serde_json::Value = serde_json::from_str(&fold).unwrap();
        assert_eq!(document["file_spec"], 1.2);
        assert_eq!(document["frame_unit"], "mm");
        assert_eq!(document["frame_classes"][0], "creasePattern");
        let vertices = document["vertices_coords"].as_array().unwrap();
        assert!(vertices.iter().any(|vertex| {
            vertex[0].as_f64() == Some(90.0) && vertex[1].as_f64() == Some(90.0)
        }));
        let assignments = document["edges_assignment"].as_array().unwrap();
        assert_eq!(
            assignments
                .iter()
                .filter(|assignment| assignment.as_str() == Some("U"))
                .count(),
            4
        );
    }

    #[test]
    fn fold_export_marks_kirigami_cuts_with_fold_1_2_cut_assignment() {
        let mut model = PaperModel::rectangle(2.0, 1.0).unwrap();
        model
            .split_panel_with_segment(
                PanelId(0),
                Point2::new(0.0, -0.5),
                Point2::new(0.0, 0.5),
                OperationKind::Cut,
            )
            .unwrap();

        let fold = export_fold(&model.flat_pattern_snapshot().unwrap(), 180.0).unwrap();
        let document: serde_json::Value = serde_json::from_str(&fold).unwrap();
        assert!(
            document["frame_attributes"]
                .as_array()
                .unwrap()
                .iter()
                .any(|attribute| attribute.as_str() == Some("cuts"))
        );
        assert!(
            document["edges_assignment"]
                .as_array()
                .unwrap()
                .iter()
                .any(|assignment| assignment.as_str() == Some("C"))
        );
    }
}
