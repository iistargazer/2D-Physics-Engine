// ============================================================================
// showcase.js — application wiring.
//
// Owns the Emscripten runtime object, scene navigation, all input, the live
// tuning controls, the first-run hint and the frame loop. Rendering lives in
// render.js and every measurement the panel shows lives in instrument.js.
//
// The loop is deliberately NOT emscripten_set_main_loop: the host shell decides
// when to tick, with requestAnimationFrame when the browser schedules frames and
// a timer cadence when it does not (see shell.html). A demo that renders zero
// frames in a headless or backgrounded tab is worse than one driven by a timer.
// ============================================================================
document.querySelectorAll('button').forEach(b => b.disabled = true);
// The two recovery actions must survive the blanket disable above.
$('firstRunDismiss').disabled = false;
$('reloadBtn').disabled = false;

var Module = {
  onRuntimeInitialized() {
    fitCanvas();
    setupScenes();
    showScene(0);
    runtimeReady = true;
    document.querySelectorAll('button').forEach(b => b.disabled = false);
    $('iterations').disabled = false;
    $('gravity').disabled = false;
    syncTools();
    pressed($('slow'), false);
    Instrument.sync(true);
    startLoop();
  },
  onAbort() {
    runtimeReady = false;
    document.querySelectorAll('button').forEach(b => b.disabled = true);
    $('iterations').disabled = true;
    $('gravity').disabled = true;
    $('reloadBtn').disabled = false;          // the one action that still helps
    $('runtimeStatus').textContent = 'Engine unavailable';
    $('runtimeStatus').classList.add('error');
    $('runtimeError').hidden = false;
    Instrument.setState('OFFLINE', 'idle');   // the dashboard owns that stamp
    $('toolHint').textContent = 'Reload the page over HTTP. index.js and index.wasm must sit beside index.html.';
  },
  framePtr: () => Module._frame_buffer(),
  frameCount: () => Module._frame_size(),
  debugOn: () => Module._debug_enabled() === 1,
  drawFrame: (ptr, count, debug) => {
    const frame = parseFrame(ptr, count);
    if (frame.scene !== currentScene) { showScene(frame.scene); Instrument.sync(true); }
    paintFrame(frame, debug);
    Instrument.update(frame);
    syncChrome(debug);
  }
};

// State that the engine owns but that is not a measurement: pressed styling on
// the transport controls, the run status, and which experiment is selected.
// Kept out of Instrument so the dashboard only ever reports numbers.
function syncChrome(debug) {
  const paused = Module._is_paused() === 1;
  pressed($('pause'), paused);
  $('pause').textContent = paused ? 'Resume' : 'Pause';
  pressed($('debug'), debug);
  $('runtimeStatus').textContent = paused ? 'PAUSED' : slowOn ? 'RUNNING / ¼ SPEED' : 'RUNNING';
  [...sceneBar.children].forEach((b, index) => pressed(b, index === currentScene));
  $('cadence').textContent = cadence + '.';
}

// --- scene navigation --------------------------------------------------------
function setupScenes() {
  SCENES.forEach((scene, i) => {
    const b = document.createElement('button');
    b.innerHTML = '<span class="number">' + String(i + 1).padStart(2, '0') + '</span><span>' + scene.name + '</span>';
    b.setAttribute('aria-label', (i === 9 ? '0' : i + 1) + '. ' + scene.name);
    b.onclick = () => selectScene(i);
    sceneBar.appendChild(b);
  });
}
function showScene(scene) {
  currentScene = scene;
  const info = SCENES[scene];
  const [x, y, width] = info.camera;
  camera = {x, y, width};
  $('sceneTitle').textContent = info.name;
  $('sceneCategory').textContent = info.category;
  $('note').textContent = info.description;
  $('observe').textContent = info.observe;
  $('evidence').textContent = info.evidence;
  $('cameraLabel').textContent = scene === 5 ? 'FOLLOW CHASSIS / METRES' : 'WORLD VIEW / METRES';
}
function selectScene(scene) {
  endDrag();
  spawnMode = null;
  syncTools();
  showScene(scene);
  Module._select_scene(scene);
  Instrument.sync(true);   // a new world brings new solver config and new history
  flashStage();
}
// Acknowledgement that the world was replaced. Suppressed for reduced motion.
function flashStage() {
  if (prefersReducedMotion()) return;
  const wipe = $('stageWipe');
  wipe.classList.remove('on');
  void wipe.offsetWidth;   // force a reflow so the animation restarts
  wipe.classList.add('on');
}

// --- tools and dragging ------------------------------------------------------
function syncTools() {
  pressed($('spawnBox'), spawnMode === 'box');
  pressed($('spawnCircle'), spawnMode === 'circle');
  if (spawnMode) canvas.dataset.tool = spawnMode; else delete canvas.dataset.tool;
  $('toolHint').textContent = spawnMode
    ? 'Click to place a ' + spawnMode + '. Select the tool again to return to dragging.'
    : 'Drag a body to disturb the system.';
}
function endDrag() {
  if (dragPointer === null) return;
  Module._end_drag();
  const p = dragPointer;
  dragPointer = null;
  if (canvas.hasPointerCapture(p)) canvas.releasePointerCapture(p);
  syncTools();
}
function setSpawnMode(mode) {
  endDrag();
  spawnMode = spawnMode === mode ? null : mode;
  syncTools();
}

// --- controls ----------------------------------------------------------------
$('pause').onclick = () => Module._toggle_pause();
$('step').onclick = () => Module._step_once();
$('restart').onclick = () => selectScene(currentScene);
$('debug').onclick = () => Module._toggle_debug();
$('slow').onclick = () => { slowOn = !slowOn; Module._set_slow(slowOn ? .25 : 1); pressed($('slow'), slowOn); };
$('spawnBox').onclick = () => setSpawnMode('box');
$('spawnCircle').onclick = () => setSpawnMode('circle');
$('reloadBtn').onclick = () => location.reload();

// Live solver tuning. Each control writes through to the running world and then
// re-reads the engine, so the UI can never disagree with the solver's own state.
$('iterations').oninput = e => {
  if (!runtimeReady) return;
  const v = Number(e.target.value);
  Module._set_iterations(v);
  $('iterationsVal').textContent = String(v);
};
$('gravity').oninput = e => {
  if (!runtimeReady) return;
  const v = Number(e.target.value);
  Module._set_gravity(v);
  $('gravityVal').textContent = v.toFixed(2);
};
$('ngs').onclick = () => {
  if (!runtimeReady) return;
  Module._set_position_correction(Module._position_correction() === 1 ? 0 : 1);
  Instrument.sync(false);
};
$('sleeping').onclick = () => {
  if (!runtimeReady) return;
  Module._set_sleeping(Module._sleeping_enabled() === 1 ? 0 : 1);
  Instrument.sync(false);
};
$('profile').onclick = () => {
  if (!runtimeReady) return;
  Module._set_profile(Module._profile_enabled() === 1 ? 0 : 1);
  Instrument.sync(false);
};

// --- pointer ----------------------------------------------------------------
canvas.addEventListener('pointerdown', e => {
  if (!runtimeReady || e.button !== 0 || dragPointer !== null) return;
  const [x, y] = toWorld(...canvasPoint(e));
  if (spawnMode === 'box') Module._spawn_box(x, y, Math.random() * 1.2 - .6);
  else if (spawnMode === 'circle') Module._spawn_circle(x, y);
  else if (Module._begin_drag(x, y)) {
    dragPointer = e.pointerId;
    canvas.setPointerCapture(e.pointerId);
    $('toolHint').textContent = 'Body held. Move the pointer, then release.';
  }
});
canvas.addEventListener('pointermove', e => { if (e.pointerId === dragPointer) Module._update_drag(...toWorld(...canvasPoint(e))); });
canvas.addEventListener('pointerup', e => { if (e.pointerId === dragPointer) endDrag(); });
canvas.addEventListener('pointercancel', e => { if (e.pointerId === dragPointer) endDrag(); });
canvas.addEventListener('lostpointercapture', endDrag);
window.addEventListener('blur', endDrag);

// --- keyboard ---------------------------------------------------------------
window.addEventListener('keydown', e => {
  if (!runtimeReady) return;
  if (e.key >= '1' && e.key <= '9') selectScene(Number(e.key) - 1);
  else if (e.key === '0') selectScene(9);
  else if (e.key === ' ') { if (e.target instanceof HTMLButtonElement) return; Module._toggle_pause(); e.preventDefault(); }
  else if (e.key === 'n') Module._step_once();
  else if (e.key === 'd') Module._toggle_debug();
  else if (e.key === 'g') $('ngs').click();
  else if (e.key === 's') $('sleeping').click();
  else if (e.key === 'p') $('profile').click();
});

// --- first run --------------------------------------------------------------
// Guidance for someone who has just opened the page. Dismissal is remembered, so
// the hint costs a returning visitor nothing.
(function initFirstRun() {
  let seen = false;
  try { seen = localStorage.getItem('phys2d.firstRun') === 'done'; } catch (err) { seen = false; }
  if (seen) $('firstRun').classList.add('hidden');
  $('firstRunDismiss').onclick = () => {
    $('firstRun').classList.add('hidden');
    try { localStorage.setItem('phys2d.firstRun', 'done'); } catch (err) { /* private mode */ }
  };
})();

// --- frame loop -------------------------------------------------------------
window.addEventListener('resize', fitCanvas);
function startLoop() {
  let rafWorks = false;
  requestAnimationFrame(() => { rafWorks = true; });
  setTimeout(() => {
    cadence = rafWorks ? 'Animation-frame cadence' : 'Timer cadence';
    if (rafWorks) (function frame() { Module._tick_js(); requestAnimationFrame(frame); })();
    else (function frame() { Module._tick_js(); setTimeout(frame, 1000 / 60); })();
  }, 250);
}
