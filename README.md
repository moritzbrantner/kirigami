# kirigami

Interactive kirigami geometry and simulation for the web, with authoritative Rust semantics compiled to WebAssembly.

## Architecture

- `crates/kirigami-core`: authoritative paper semantics plus a half-edge planar subdivision for faces, boundaries, cuts, creases, connectivity, fold transforms, and renderer-neutral snapshots.
- `crates/kirigami-export`: deterministic adapter from authoritative flat-pattern snapshots to print-ready PDF and physical-size SVG. It does not infer cut/crease meaning.
- `crates/kirigami-wasm`: thin WebAssembly boundary. It does not own geometry or export rules.
- `web`: presentation-only browser demo suitable for GitHub Pages.
- `3d-lab / three-d-core`: shared renderer-neutral 3D mesh vocabulary, consumed at a pinned revision rather than copied locally.

`PlanarTopology` is the 2D geometric authority. It keeps stable face/half-edge relationships, splits shared edges consistently when later operations meet an existing crease, rejects chords that leave a face, and triangulates simple faces deterministically without assuming a convex fan. Cut versus crease meaning remains a kirigami domain concern layered on that subdivision: a geometric twin does not by itself mean the paper remains materially connected.

The current domain supports straight and polyline cuts, holes, bridges, and straight multi-crease fold states. The topology layer is intentionally reusable for arbitrary simple faces so constrained triangulation quality upgrades, collision queries, and printable/export adapters do not require moving authority into the UI or renderer.

The browser can export the current flat pattern as PDF or SVG at an exact requested physical width. PDF output is single-page vector geometry with solid material/cut lines and dashed crease lines, suitable for printing at 100% scale.

The PR validation workflow checks formatting, clippy, tests, and the WASM target at the exact proposed head before integration.

## Local validation

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo build -p kirigami-wasm --target wasm32-unknown-unknown --release
```

To build the browser package, install `wasm-pack` and run:

```sh
wasm-pack build crates/kirigami-wasm --target web --release --out-dir ../../dist/pkg
cp web/index.html web/app.js web/style.css dist/
```

See `ROADMAP.md` for the topology, collision, renderer, and constraint-solver progression.
