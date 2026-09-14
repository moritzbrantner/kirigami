use kirigami_core::{FoldRequest, OperationKind, PanelId, PaperModel, Point2};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn demo_snapshot(angle_degrees: f32, mode: &str) -> Result<String, JsValue> {
    let kind = match mode {
        "crease" => OperationKind::Crease,
        "cut" => OperationKind::Cut,
        _ => return Err(JsValue::from_str("mode must be 'crease' or 'cut'")),
    };

    let mut model = PaperModel::rectangle(2.4, 1.5).map_err(js_error)?;
    let seam = model
        .split_panel_with_segment(
            PanelId(0),
            Point2::new(0.0, -0.75),
            Point2::new(0.0, 0.75),
            kind,
        )
        .map_err(js_error)?;
    let fold = (kind == OperationKind::Crease).then_some(FoldRequest {
        seam,
        angle_radians: angle_degrees.to_radians(),
    });
    let snapshot = model.render_snapshot(fold).map_err(js_error)?;
    serde_json::to_string(&snapshot).map_err(js_error)
}

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}
