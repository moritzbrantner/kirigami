use kirigami_core::{FoldRequest, OperationKind, PanelId, PaperModel, Point2};
use kirigami_export::{PdfExportOptions, export_pdf, export_svg};
use kirigami_targets::load_target;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn demo_snapshot(angle_degrees: f32, mode: &str) -> Result<String, JsValue> {
    let (model, folds) = build_demo(angle_degrees, mode)?;
    let snapshot = model.render_snapshot_with_folds(&folds).map_err(js_error)?;
    serde_json::to_string(&snapshot).map_err(js_error)
}

#[wasm_bindgen]
pub fn demo_pdf(
    mode: &str,
    template_width_mm: f32,
    page_width_mm: f32,
    page_height_mm: f32,
) -> Result<Vec<u8>, JsValue> {
    let (model, _) = build_demo(0.0, mode)?;
    let pattern = model.flat_pattern_snapshot().map_err(js_error)?;
    export_pdf(
        &pattern,
        PdfExportOptions {
            page_width_mm,
            page_height_mm,
            margin_mm: 10.0,
            template_width_mm,
        },
    )
    .map_err(js_error)
}

#[wasm_bindgen]
pub fn demo_svg(mode: &str, template_width_mm: f32) -> Result<String, JsValue> {
    let (model, _) = build_demo(0.0, mode)?;
    let pattern = model.flat_pattern_snapshot().map_err(js_error)?;
    export_svg(&pattern, template_width_mm).map_err(js_error)
}

#[wasm_bindgen]
pub fn target_snapshot(file_name: &str, bytes: &[u8]) -> Result<String, JsValue> {
    let target = load_target(file_name, bytes).map_err(js_error)?;
    let snapshot = target.snapshot().map_err(js_error)?;
    serde_json::to_string(&snapshot).map_err(js_error)
}

fn build_demo(angle_degrees: f32, mode: &str) -> Result<(PaperModel, Vec<FoldRequest>), JsValue> {
    let mut model = PaperModel::rectangle(2.4, 1.5).map_err(js_error)?;
    let folds = match mode {
        "crease" => {
            let operation = model
                .split_panel_with_segment(
                    PanelId(0),
                    Point2::new(0.0, -0.75),
                    Point2::new(0.0, 0.75),
                    OperationKind::Crease,
                )
                .map_err(js_error)?;
            vec![FoldRequest {
                operation,
                angle_radians: angle_degrees.to_radians(),
            }]
        }
        "cut" => {
            model
                .split_panel_with_segment(
                    PanelId(0),
                    Point2::new(0.0, -0.75),
                    Point2::new(0.0, 0.75),
                    OperationKind::Cut,
                )
                .map_err(js_error)?;
            Vec::new()
        }
        "hole" => {
            model
                .cut_closed_path(
                    PanelId(0),
                    &[
                        Point2::new(-0.45, -0.35),
                        Point2::new(0.45, -0.35),
                        Point2::new(0.45, 0.35),
                        Point2::new(-0.45, 0.35),
                    ],
                )
                .map_err(js_error)?;
            Vec::new()
        }
        "bridge" => {
            model
                .cut_closed_path(
                    PanelId(0),
                    &[
                        Point2::new(-0.45, -0.35),
                        Point2::new(0.45, -0.35),
                        Point2::new(0.45, 0.35),
                        Point2::new(-0.45, 0.35),
                    ],
                )
                .map_err(js_error)?;
            model
                .cut_boundary_bridge(
                    PanelId(0),
                    &[Point2::new(-1.2, 0.0), Point2::new(-0.45, 0.0)],
                )
                .map_err(js_error)?;
            Vec::new()
        }
        "accordion" => {
            let first = model
                .split_across_panels_with_polyline(
                    &[Point2::new(-0.4, -0.75), Point2::new(-0.4, 0.75)],
                    OperationKind::Crease,
                )
                .map_err(js_error)?;
            let second = model
                .split_across_panels_with_polyline(
                    &[Point2::new(0.4, -0.75), Point2::new(0.4, 0.75)],
                    OperationKind::Crease,
                )
                .map_err(js_error)?;
            vec![
                FoldRequest {
                    operation: first,
                    angle_radians: angle_degrees.to_radians(),
                },
                FoldRequest {
                    operation: second,
                    angle_radians: -angle_degrees.to_radians(),
                },
            ]
        }
        _ => {
            return Err(JsValue::from_str(
                "mode must be 'crease', 'cut', 'hole', 'bridge', or 'accordion'",
            ));
        }
    };
    Ok((model, folds))
}

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}
