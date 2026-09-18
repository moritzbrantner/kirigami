import init, { demo_snapshot, target_snapshot } from "./pkg/kirigami_wasm.js";

const canvas = document.querySelector("#canvas");
const context = canvas.getContext("2d");
const mode = document.querySelector("#mode");
const angle = document.querySelector("#angle");
const angleOutput = document.querySelector("#angle-output");
const status = document.querySelector("#status");
const targetFile = document.querySelector("#target-file");
const targetStatus = document.querySelector("#target-status");
const clearTarget = document.querySelector("#clear-target");

const MAX_PREVIEW_TRIANGLES = 25000;
let activeTarget = null;

await init();

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const scale = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * scale));
  const height = Math.max(1, Math.round(rect.height * scale));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  context.setTransform(scale, 0, 0, scale, 0, 0);
  render();
}

function createProjector(vertices) {
  const rect = canvas.getBoundingClientRect();
  if (vertices.length === 0) {
    return () => [rect.width * 0.5, rect.height * 0.5];
  }

  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  for (const [x, y, z] of vertices) {
    const projectedX = x - z * 0.55;
    const projectedY = -(y + z * 0.25);
    minX = Math.min(minX, projectedX);
    maxX = Math.max(maxX, projectedX);
    minY = Math.min(minY, projectedY);
    maxY = Math.max(maxY, projectedY);
  }

  const spanX = Math.max(maxX - minX, 1e-4);
  const spanY = Math.max(maxY - minY, 1e-4);
  const padding = Math.min(36, rect.width * 0.08, rect.height * 0.08);
  const scale = Math.min(
    (rect.width - padding * 2) / spanX,
    (rect.height - padding * 2) / spanY,
  );
  const centerX = (minX + maxX) * 0.5;
  const centerY = (minY + maxY) * 0.5;

  return ([x, y, z]) => [
    rect.width * 0.5 + (x - z * 0.55 - centerX) * scale,
    rect.height * 0.5 + (-(y + z * 0.25) - centerY) * scale,
  ];
}

function clearCanvas() {
  const rect = canvas.getBoundingClientRect();
  context.clearRect(0, 0, rect.width, rect.height);
  context.lineJoin = "round";
  context.setLineDash([]);
}

function renderPaper() {
  const currentMode = mode.value;
  const currentAngle = Number(angle.value);
  angle.disabled = currentMode !== "crease";
  angleOutput.value = `${currentAngle}°`;

  const snapshot = JSON.parse(demo_snapshot(currentAngle, currentMode));
  const project = createProjector(snapshot.vertices);

  for (let offset = 0; offset < snapshot.indices.length; offset += 3) {
    const points = snapshot.indices
      .slice(offset, offset + 3)
      .map((index) => project(snapshot.vertices[index]));
    context.beginPath();
    context.moveTo(...points[0]);
    context.lineTo(...points[1]);
    context.lineTo(...points[2]);
    context.closePath();
    context.fillStyle = offset % 6 === 0 ? "#f4ead7" : "#e7d7bd";
    context.fill();
    context.strokeStyle = "#342d25";
    context.lineWidth = 1.2;
    context.stroke();
  }

  for (const seam of snapshot.seams) {
    const start = project(seam.start);
    const end = project(seam.end);
    context.beginPath();
    context.moveTo(...start);
    context.lineTo(...end);
    context.setLineDash(seam.kind === "crease" ? [7, 6] : []);
    context.strokeStyle = seam.kind === "crease" ? "#5e6575" : "#a23232";
    context.lineWidth = 2.5;
    context.stroke();
  }
  context.setLineDash([]);

  status.textContent = `${snapshot.panel_count} panels · ${snapshot.component_count} connected component${snapshot.component_count === 1 ? "" : "s"}`;
}

function renderTarget() {
  angle.disabled = true;
  const project = createProjector(activeTarget.vertices);

  if (activeTarget.kind === "mesh") {
    const indexLimit = Math.min(
      activeTarget.indices.length,
      MAX_PREVIEW_TRIANGLES * 3,
    );
    for (let offset = 0; offset < indexLimit; offset += 3) {
      const points = activeTarget.indices
        .slice(offset, offset + 3)
        .map((index) => project(activeTarget.vertices[index]));
      context.beginPath();
      context.moveTo(...points[0]);
      context.lineTo(...points[1]);
      context.lineTo(...points[2]);
      context.closePath();
      context.fillStyle = "#e9e1d2";
      context.fill();
      context.strokeStyle = "#514a41";
      context.lineWidth = 0.8;
      context.stroke();
    }
    const clipped = activeTarget.triangle_count > MAX_PREVIEW_TRIANGLES;
    const area = activeTarget.measurements.mesh_surface_area.toFixed(3);
    status.textContent = `Target mesh · ${activeTarget.mesh_count} mesh${activeTarget.mesh_count === 1 ? "" : "es"} · ${activeTarget.vertices.length} vertices · ${activeTarget.triangle_count} triangles · normalized area ${area}${clipped ? " · preview capped at 25,000 triangles" : ""}`;
    return;
  }

  context.strokeStyle = "#514a41";
  context.lineWidth = 3;
  for (const [parent, child] of activeTarget.edges) {
    context.beginPath();
    context.moveTo(...project(activeTarget.vertices[parent]));
    context.lineTo(...project(activeTarget.vertices[child]));
    context.stroke();
  }

  for (const vertex of activeTarget.vertices) {
    const [x, y] = project(vertex);
    context.beginPath();
    context.arc(x, y, 5, 0, Math.PI * 2);
    context.fillStyle = "#f7f3ec";
    context.fill();
    context.strokeStyle = "#342d25";
    context.lineWidth = 2;
    context.stroke();
  }
  const length = activeTarget.measurements.skeleton_total_edge_length.toFixed(3);
  status.textContent = `Target skeleton · ${activeTarget.joint_count} joints · normalized bone length ${length}`;
}

function render() {
  clearCanvas();
  if (activeTarget) {
    renderTarget();
  } else {
    renderPaper();
  }
}

targetFile.addEventListener("change", async () => {
  const [file] = targetFile.files;
  if (!file) {
    return;
  }

  targetStatus.textContent = `Loading ${file.name}…`;
  try {
    const bytes = new Uint8Array(await file.arrayBuffer());
    activeTarget = JSON.parse(target_snapshot(file.name, bytes));
    clearTarget.disabled = false;
    const fingerprint = activeTarget.source_fingerprint.split(":").at(-1).slice(0, 8);
    targetStatus.textContent =
      activeTarget.kind === "mesh"
        ? `${file.name} loaded as a normalized 3D mesh · ${fingerprint}`
        : `${file.name} loaded as a normalized skeleton · ${fingerprint}`;
    render();
  } catch (error) {
    activeTarget = null;
    clearTarget.disabled = true;
    targetStatus.textContent = error instanceof Error ? error.message : String(error);
    render();
  }
});

clearTarget.addEventListener("click", () => {
  activeTarget = null;
  targetFile.value = "";
  targetStatus.textContent = "No target loaded.";
  clearTarget.disabled = true;
  render();
});

mode.addEventListener("change", render);
angle.addEventListener("input", render);
window.addEventListener("resize", resizeCanvas);
resizeCanvas();
