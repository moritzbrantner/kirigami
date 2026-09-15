from pathlib import Path

path = Path("crates/kirigami-core/src/lib.rs")
text = path.read_text()
old = '''    let neighbors: HashSet<(PanelId, PanelId)> = seams
        .iter()
        .filter(|seam| seam.panel_a != seam.panel_b)
        .map(|seam| canonical_panel_pair(seam.panel_a, seam.panel_b))
        .collect();
'''
if old in text:
    text = text.replace(old, "", 1)
path.write_text(text)
