use kirigami_core::{FoldRequest, OperationKind, PanelId, PaperModel, Point2};
use kirigami_targets::load_target;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn demo_snapshot(angle_degrees: f32, mode: &str) -> Result<String, JsValue> {
    let mut model = PaperModel::rectangle(2.4, 1.5).map_err(js_error)?;
    let fold = match mode {
        "crease" => {
            let operation = model
                .split_panel_with_segment(
                    PanelId(0),
                    Point2::new(0.0, -0.75),
                    Point2::new(0.0, 0.75),
                    OperationKind::Crease,
                )
                .map_err(js_error)?;
            Some(FoldRequest {
                operation,
                angle_radians: angle_degrees.to_radians(),
            })
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
            None
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
            None
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
            None
        }
        _ => {
            return Err(JsValue::from_str(
                "mode must be 'crease', 'cut', 'hole', or 'bridge'",
            ));
        }
    };
    let snapshot = model.render_snapshot(fold).map_err(js_error)?;
    serde_json::to_string(&snapshot).map_err(js_error)
}

#[wasm_bindgen]
pub fn target_snapshot(file_name: &str, bytes: &[u8]) -> Result<String, JsValue> {
    let target = load_target(file_name, bytes).map_err(js_error)?;
    let snapshot = target.snapshot().map_err(js_error)?;
    serde_json::to_string(&snapshot).map_err(js_error)
}

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}
