# kirigami

Interactive kirigami geometry and simulation for the web, with authoritative Rust semantics compiled to WebAssembly.

## Architecture

- `crates/kirigami-core`: authoritative sheet topology, cut/crease semantics, connectivity, fold transforms, and renderer-neutral snapshots.
- `crates/kirigami-wasm`: thin WebAssembly boundary. It does not own geometry rules.
- `web`: presentation-only browser demo suitable for GitHub Pages.
- `3d-lab / three-d-core`: shared renderer-neutral 3D mesh vocabulary, consumed at a pinned revision rather than copied locally.

The first topology primitive splits a convex panel with a straight boundary-to-boundary cut or crease. This is intentionally narrower than the long-term model: the public domain boundary is established first, then the implementation can move to half-edge planar subdivision and constrained triangulation without shifting authority into the UI or renderer.

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
