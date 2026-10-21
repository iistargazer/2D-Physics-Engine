// ============================================================================
// render.js â€” everything that touches the main canvas.
//
// Two pure-ish halves:
//   parseFrame(ptr, count)  decodes the flat float buffer the engine wrote.
//   paintFrame(frame, debug) draws it through the shared camera transform.
//
// Keeping parsing separate from painting lets the dashboard read the exact same
// decoded numbers the renderer drew, instead of re-decoding the buffer a second
// time (or, worse, reading its own DOM text back).
//
// Buffer layout is documented in web/main.cpp.
// ============================================================================

// Dormant bodies are outlined with a dash as well as drawn in the dormant hue, so
// "awake or asleep" is never carried by colour alone on this page.
const DASH_ASLEEP = [3, 3];
const DASH_NONE = [];

function parseFrame(ptr, count) {
  const data = Module.HEAPF32.subarray(ptr >> 2, (ptr >> 2) + count);
  const frame = {
    scene: data[3] | 0,
    bodies: data[0] | 0,
    contacts: data[1] | 0,
    stepMs: data[2],
    awake: data[4] | 0,
    asleep: 0, staticCount: 0,
    list: []
  };
  let i = 5;
  for (let b = 0; b < frame.bodies; b++) {
    const kind = data[i++], color = data[i++], x = data[i++], y = data[i++], extra = data[i++];
    const verts = [];
    if (kind === 1) for (let v = 0; v < extra; v++) verts.push([data[i++], data[i++]]);
    if (color === 0) frame.staticCount++;
    else if (color === 2) frame.asleep++;
    frame.list.push({kind, color, x, y, extra, verts});
  }
  frame.contactOffset = i;   // where the contact block starts, for the debug draw
  return frame;
}

function paintFrame(frame, debug) {
  const scene = frame.scene, bodies = frame.list;
  const data = Module.HEAPF32;
  const ptr = Module._frame_buffer() >> 2;
  // Car camera: follow the chassis, but never while a body is held â€” moving the
  // camera under the pointer would drag its target away from the cursor.
  if (scene === 5 && dragPointer === null && bodies[4]) camera.x = bodies[4].x;

  const s = viewportScale();
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Sparse metre grid gives scale without disguising the simulated geometry.
  const grid = scene === 7 ? 25 : (scene === 2 || scene === 4) ? 5 : 1;
  const [left, top] = toWorld(0, 0), [right, bottom] = toWorld(canvas.width, canvas.height);
  ctx.strokeStyle = '#262f4d'; ctx.lineWidth = 1; ctx.beginPath();
  for (let x = Math.ceil(left / grid) * grid; x <= right; x += grid) {
    const p = toScreen(x, 0); ctx.moveTo(p[0], 0); ctx.lineTo(p[0], canvas.height);
  }
  for (let y = Math.ceil(bottom / grid) * grid; y <= top; y += grid) {
    const p = toScreen(0, y); ctx.moveTo(0, p[1]); ctx.lineTo(canvas.width, p[1]);
  }
  ctx.stroke();

  // Constraint guides: presentation only, not extra physics.
  if (scene === 0 || scene === 2) {
        // A constraint guide is structure, not energy or state, so it is drawn from the
    // ink ramp rather than from one of the four meaning hues — and bright enough to
    // clear 3:1 on the sky behind it (docs/VISUAL.md §7).
    ctx.strokeStyle = '#a6a3b8aa'; ctx.lineWidth = 1.5; ctx.beginPath();
    if (scene === 0) for (let b = 1; b < 6 && bodies[b]; b++) {
      const a = toScreen(b - 3, 3), p = toScreen(bodies[b].x, bodies[b].y);
      ctx.moveTo(...a); ctx.lineTo(...p);
    }
    else if (bodies[212]) {
      const a = toScreen(-6, 24), p = toScreen(bodies[212].x, bodies[212].y);
      ctx.moveTo(...a); ctx.lineTo(...p);
    }
    ctx.stroke();
  }

  let dashed = null;
  bodies.forEach((b, index) => {
    // Set the dash only when it changes: allocating a dash array per body per
    // frame would be pointless churn in a renderer this cheap.
    const wantDash = b.color === 2;
    if (wantDash !== dashed) { ctx.setLineDash(wantDash ? DASH_ASLEEP : DASH_NONE); dashed = wantDash; }
    ctx.fillStyle = scene === 8 && index === 1 ? '#e08b62' : COLORS[b.color];
    ctx.strokeStyle = '#efe6d580'; ctx.lineWidth = 1;
    ctx.beginPath();
    if (b.kind === 0) { const [x, y] = toScreen(b.x, b.y); ctx.arc(x, y, b.extra * s, 0, Math.PI * 2); }
    else { b.verts.forEach(([x, y], v) => { const p = toScreen(x, y); v ? ctx.lineTo(...p) : ctx.moveTo(...p); }); ctx.closePath(); }
    ctx.fill(); ctx.stroke();
  });

  if (dashed) ctx.setLineDash(DASH_NONE);   // leave the context clean for the debug pass

  if (debug) {
    let i = frame.contactOffset;
    for (let c = 0; c < frame.contacts; c++) {
      const x = data[ptr + i++], y = data[ptr + i++], nx = data[ptr + i++], ny = data[ptr + i++], impulse = data[ptr + i++];
      const p = toScreen(x, y);
      ctx.fillStyle = '#f2cb8e';
      ctx.beginPath(); ctx.arc(...p, Math.max(2, canvas.width / 640), 0, Math.PI * 2); ctx.fill();
      const len = (0.15 + Math.min(0.6, impulse * 0.03)) * s;
      ctx.strokeStyle = '#e08b62';
      ctx.beginPath(); ctx.moveTo(...p); ctx.lineTo(p[0] + nx * len, p[1] - ny * len); ctx.stroke();
    }
  }
}
