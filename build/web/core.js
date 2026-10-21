// ============================================================================
// core.js — shared DOM handles, camera transform and tiny helpers.
//
// Loaded first. Everything here is plain global-scope script state, which is
// deliberate: the Emscripten glue expects `Module` to be a real global, so the
// whole frontend stays classic scripts rather than ES modules.
//
// The camera transform is the single source of truth shared by the renderer and
// by pointer picking, so a click always lands where the body was drawn. The body
// palette and the frame budget are defined here once for the same reason: the
// renderer paints COLORS and the instrument derives every budget number from
// BUDGET_MS / GAUGE_FULL_MS instead of repeating a literal in a label.
// ============================================================================
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const sceneBar = document.getElementById('scenes');
const $ = id => document.getElementById(id);

// Application state. Mutated by input in showcase.js, read by render.js.
let runtimeReady = false, spawnMode = null, dragPointer = null, currentScene = 0, slowOn = false;
// Camera coordinates are [x, y, visibleWidth] in metres.
let camera = {x: 0, y: 2.2, width: 9};
let cadence = 'starting';

// Body colors by engine state: 0 static, 1 awake, 2 asleep. These are the same
// three hues the dashboard's telemetry ledger uses — teal for frozen geometry,
// amber for energy, lavender for a dormant island — so the picture and the panel
// name the same state with the same hue, staged a little darker here so a field
// of thousands of bodies does not glare. A static body also has to clear 3:1 on
// the lightest sky the stage paints, which is what sets its value.
// The CCD bullet is drawn from the rust ramp instead (see render.js).
const COLORS = ['#55838c', '#dba45c', '#7d7aa8'];

// The step budget the dashboard gauges against: one 60 Hz frame, and the span its
// bar covers. Everything numeric that names the budget — the gauge's full-bar
// scale, the history plot's reference line, its axis label, the static caption
// under the bar and the gauge's accessible name — is derived from these two
// numbers, so no label can disagree with the bar it labels.
const BUDGET_MS = 1000 / 60;
const GAUGE_FULL_MS = BUDGET_MS * 2;

function viewportScale() { return canvas.width / camera.width; }
function toScreen(x, y) {
  const s = viewportScale();
  return [canvas.width / 2 + (x - camera.x) * s, canvas.height / 2 - (y - camera.y) * s];
}
function toWorld(px, py) {
  const s = viewportScale();
  return [camera.x + (px - canvas.width / 2) / s, camera.y - (py - canvas.height / 2) / s];
}
function canvasPoint(e) {
  const r = canvas.getBoundingClientRect(), style = getComputedStyle(canvas);
  const l = parseFloat(style.borderLeftWidth), rgt = parseFloat(style.borderRightWidth);
  const t = parseFloat(style.borderTopWidth), b = parseFloat(style.borderBottomWidth);
  return [(e.clientX - r.left - l) * canvas.width / (r.width - l - rgt),
          (e.clientY - r.top - t) * canvas.height / (r.height - t - b)];
}
function fitCanvas() {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  canvas.width = Math.max(640, Math.floor(canvas.clientWidth * dpr));
  canvas.height = Math.round(canvas.width * 9 / 16);
}
function pressed(button, on) {
  button.classList.toggle('on', on);
  button.setAttribute('aria-pressed', String(on));
}
// Step costs span microseconds to tens of milliseconds, so the unit follows the
// magnitude instead of padding everything with zeros.
function fmtMs(ms) {
  if (!(ms > 0)) return '0.00 ms';
  return ms >= 0.01 ? ms.toFixed(2) + ' ms' : (ms * 1000).toFixed(1) + ' µs';
}
function prefersReducedMotion() {
  return typeof matchMedia === 'function' && matchMedia('(prefers-reduced-motion: reduce)').matches;
}
