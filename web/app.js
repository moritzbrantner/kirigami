import init, { demo_snapshot } from "./pkg/kirigami_wasm.js";

const canvas = document.querySelector("#canvas");
const context = canvas.getContext("2d");
const mode = document.querySelector("#mode");
const angle = document.querySelector("#angle");
const angleOutput = document.querySelector("#angle-output");
const status = document.querySelector("#status");

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

function project([x, y, z]) {
  const width = canvas.getBoundingClientRect().width;
  const height = canvas.getBoundingClientRect().height;
  const scale = Math.min(width / 3.4, height / 2.3);
  return [
    width * 0.5 + (x - z * 0.55) * scale,
    height * 0.52 - (y + z * 0.25) * scale,
  ];
}

function render() {
  const currentMode = mode.value;
  const currentAngle = Number(angle.value);
  angle.disabled = currentMode === "cut";
  angleOutput.value = `${currentAngle}°`;

  const snapshot = JSON.parse(demo_snapshot(currentAngle, currentMode));
  const rect = canvas.getBoundingClientRect();
  context.clearRect(0, 0, rect.width, rect.height);
  context.lineJoin = "round";

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

mode.addEventListener("change", render);
angle.addEventListener("input", render);
window.addEventListener("resize", resizeCanvas);
resizeCanvas();
