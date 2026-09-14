from pathlib import Path

path = Path("crates/kirigami-core/src/topology.rs")
text = path.read_text()
old = '''        if position > start.position + PATH_POSITION_TOLERANCE
            && position < end.position - PATH_POSITION_TOLERANCE
        {
            if !approximately_equal(*fragment.last().expect("fragment has start"), point) {
                fragment.push(point);
            }
        }'''
new = '''        if position > start.position + PATH_POSITION_TOLERANCE
            && position < end.position - PATH_POSITION_TOLERANCE
            && !approximately_equal(*fragment.last().expect("fragment has start"), point)
        {
            fragment.push(point);
        }'''
if old not in text and new not in text:
    raise SystemExit("path slicing pattern not found")
path.write_text(text.replace(old, new, 1))
