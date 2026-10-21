// ============================================================================
// instrument.js — the telemetry dashboard controller.
//
// Reads the engine's additive stats buffer (see buildStats() in web/main.cpp)
// once per rendered frame and projects it onto the panel: counts, a step-cost
// gauge against the 60 Hz budget, a wake/sleep composition bar, a calibrated
// cost history, the opt-in seven-stage breakdown, and the metre ruler under the
// stage.
//
// Everything animated here is driven by a measured value. There is no decorative
// motion: the budget bar is literally the step cost, the composition bar is
// literally how many bodies are awake, the history plot is the last frames'
// exponential moving averages, and the ruler's tick spacing is the same camera
// scale the pointer inverse maps through.
// ============================================================================
const STAGES = ['broadphase', 'narrowphase', 'islands', 'solve', 'integrate', 'ccd', 'bookkeeping'];
const SPARK_MAX = 120;
// Inset of the plotted series inside the canvas bitmap, in bitmap pixels. The
// axis labels are placed from the same two numbers, so a gridline and its label
// cannot drift apart.
const SPARK_PAD_TOP = 9;
const SPARK_PAD_BOTTOM = 4;

const PHASE_NOTE_OFF =
  'Stage attribution is read from the engine\'s own per-frame timings and is off by default, '
  + 'so the default step carries no measurement overhead. Enable it under tuning.';
const PHASE_NOTE_ON =
  'Each bar is that stage\'s share of the measured total, and the seven stages sum toward the '
  + 'fixed step rather than hiding work inside one bucket. Read with the engine clock, which the '
  + 'browser coarsens.';
const PHASE_NOTE_BELOW =
  'Every stage measured below the browser clock\'s resolution this frame. That is the timer, not '
  + 'the engine: a small scene steps faster than the clock can tick.';

const phaseRows = {};
document.querySelectorAll('#phaseList .phase').forEach(row => {
  phaseRows[row.dataset.stage] = {bar: row.querySelector('b'), val: row.querySelector('em')};
});

const spark = $('spark');
const sparkCtx = spark.getContext('2d');
const stepTile = document.querySelector('[data-tile="step"]');
const axisBudget = $('axisBudget');
const axisZero = $('axisZero');

// A measured value with its unit set subordinate to the number: "30.03 ms".
function valueWithUnit(text) {
  const [num, unit] = text.split(' ');
  return '<span class="num">' + num + '</span>' + (unit ? '<i>' + unit + '</i>' : '');
}

// Signed world coordinate for the ruler's edge labels, using a real minus sign.
function metres(x) {
  return (x < 0 ? '\u2212' : '') + Math.abs(x).toFixed(2) + ' m';
}

const Instrument = {
  history: [],
  last: null,
  frames: 0,
  scaleKey: '',

  read() {
    const s = Module._stats_buffer() >> 2;
    return Module.HEAPF32.subarray(s, s + Module._stats_size());
  },

  // Mirror the engine's live configuration onto the tuning controls. Called on
  // load, after every scene change, and after every toggle, so a control can
  // never drift from the value the solver is actually using.
  sync(resetHistory) {
    const st = this.read();
    const iterations = st[8] | 0;
    $('iterations').value = String(iterations);
    $('iterationsVal').textContent = String(iterations);
    const gravity = Math.round(st[11] * 100) / 100;
    $('gravity').value = String(gravity);
    $('gravityVal').textContent = gravity.toFixed(2);
    pressed($('ngs'), st[9] !== 0);
    pressed($('sleeping'), st[10] !== 0);
    const on = st[7] !== 0;
    pressed($('profile'), on);
    $('solverMode').textContent = Module._solver_mode() === 1 ? 'TGS' : 'PGS';
    $('phasePanel').classList.toggle('off', !on);
    $('phaseSource').textContent = on ? 'PROFILE ON' : 'PROFILE OFF';
    if (!on) {
      STAGES.forEach(name => { phaseRows[name].val.textContent = '—'; phaseRows[name].bar.style.width = '0%'; });
      const total = $('stageTotal');
      total.textContent = '—';
      total.classList.remove('good', 'over');
      $('phaseNote').textContent = PHASE_NOTE_OFF;
    }
    if (resetHistory) { this.history.length = 0; this.frames = 0; }
    this.drawSpark();
    this.drawScale();
  },

  update(frame) {
    this.last = frame;
    const st = this.read();
    const stepMs = frame.stepMs;
    const over = stepMs > BUDGET_MS;
    const paused = Module._is_paused() === 1;

    $('bodyCount').textContent = frame.bodies.toLocaleString();
    $('awakeCount').textContent = frame.awake.toLocaleString();
    $('asleepCount').textContent = frame.asleep.toLocaleString();
    $('staticCount').textContent = frame.staticCount.toLocaleString();
    $('contactCount').textContent = frame.contacts.toLocaleString();
    // The one tile whose value carries a unit: the measured step.
    $('stepCost').innerHTML = valueWithUnit(fmtMs(stepMs));
    stepTile.classList.toggle('over', over);

    // Gauge: a full bar is two 60 Hz budgets, so the 60 Hz mark sits at halfway.
    $('budgetFill').style.width = (Math.min(stepMs / (BUDGET_MS * 2), 1) * 100).toFixed(1) + '%';
    $('budgetBar').classList.toggle('over', over);
    $('budgetValue').textContent = fmtMs(stepMs);
    const verdict = $('budgetVerdict');
    verdict.textContent = over ? 'OVER 60 HZ BUDGET' : 'WITHIN BUDGET';
    verdict.classList.toggle('over', over);
    verdict.classList.toggle('good', !over && stepMs > 0);

    // Composition bar: static, then asleep, then awake, as shares of all bodies.
    const total = Math.max(1, frame.bodies);
    $('segStatic').style.width = (frame.staticCount / total * 100) + '%';
    $('segAsleep').style.width = (frame.asleep / total * 100) + '%';
    $('segAwake').style.width = (frame.awake / total * 100) + '%';

    // Sleep state is inferred from the counts, not assumed.
    const dynamic = frame.awake + frame.asleep;
    let state = 'ACTIVE';
    if (paused) state = 'PAUSED';
    else if (frame.awake === 0 && dynamic > 0) state = 'SLEEPING';
    else if (frame.awake > 0 && frame.asleep > 0) state = 'SETTLING';
    $('stateText').textContent = state;
    const pill = $('statePill');
    pill.classList.toggle('idle', paused);
    pill.classList.toggle('strain', !paused && over);

    const steps = st[12] | 0;
    $('simTime').textContent = 'SIM ' + (steps / 60).toFixed(1) + ' S';
    $('stepCount').textContent = steps.toLocaleString();

    this.updatePhases(st, stepMs);
    this.drawScale();
    this.pushHistory(stepMs);
    if (this.frames++ % 2 === 0) this.drawSpark();
  },

  updatePhases(st, stepMs) {
    if (st[7] === 0) return;   // .off dims the panel and sync() owns the reset
    let sum = 0;
    for (let i = 0; i < 7; i++) sum += st[i];
    STAGES.forEach((name, i) => {
      const row = phaseRows[name];
      row.val.innerHTML = valueWithUnit(fmtMs(st[i]));
      row.bar.style.width = sum > 0 ? (st[i] / sum * 100).toFixed(1) + '%' : '0%';
    });
    const total = $('stageTotal');
    if (sum > 0) {
      total.textContent = 'Σ ' + fmtMs(sum) + ' / ' + fmtMs(stepMs);
      total.classList.add('good');
      total.classList.remove('over');
      $('phaseNote').textContent = PHASE_NOTE_ON;
    } else {
      total.textContent = 'BELOW CLOCK QUANTUM';
      total.classList.remove('good', 'over');
      $('phaseNote').textContent = PHASE_NOTE_BELOW;
    }
  },

  // The ruler under the stage. One tick is one metre-scale step, chosen so the
  // ticks never collapse into a haze, and the two edge labels are the world
  // coordinates the visible view actually spans. Written only when it changes:
  // the car's follow camera moves this every frame.
  drawScale() {
    const perMetre = canvas.clientWidth / camera.width;   // CSS px per metre
    const steps = [0.5, 1, 2, 5, 10, 20, 50, 100];
    let stepM = steps[steps.length - 1];
    for (const s of steps) if (perMetre * s >= 22) { stepM = s; break; }
    const lo = camera.x - camera.width / 2;
    const hi = camera.x + camera.width / 2;
    const key = stepM + '|' + metres(lo) + '|' + metres(hi) + '|' + perMetre.toFixed(1);
    if (key === this.scaleKey) return;
    this.scaleKey = key;
    document.documentElement.style.setProperty('--metre', (perMetre * stepM).toFixed(2) + 'px');
    $('scaleStep').textContent = 'TICKS ' + stepM + ' M';
    $('scaleLo').textContent = metres(lo);
    $('scaleHi').textContent = metres(hi);
  },

  pushHistory(ms) {
    if (!Number.isFinite(ms)) return;
    this.history.push(ms);
    if (this.history.length > SPARK_MAX) this.history.shift();
  },

  drawSpark() {
    const w = spark.width, h = spark.height;
    sparkCtx.setTransform(1, 0, 0, 1, 0, 0);
    sparkCtx.clearRect(0, 0, w, h);
    const n = this.history.length;
    let sampleMax = 0;
    for (let i = 0; i < n; i++) if (this.history[i] > sampleMax) sampleMax = this.history[i];
    // Scale to the budget even when the samples sit far below it, so the 60 Hz
    // reference line stays in the plot instead of being pushed off the top.
    const peak = Math.max(BUDGET_MS, sampleMax);

    const plotBottom = h - SPARK_PAD_BOTTOM;
    const y = v => plotBottom - (Math.min(v, peak) / peak) * (plotBottom - SPARK_PAD_TOP);
    const step = w / (SPARK_MAX - 1);
    const x = i => (SPARK_MAX - n + i) * step;   // newest sample pinned to the right

    // Axis: a zero rule, then the 60 Hz budget as the reference line, so the plot
    // answers "are we inside a frame?" rather than just "what happened".
    sparkCtx.strokeStyle = 'rgba(236,229,213,.07)'; sparkCtx.lineWidth = 1;
    sparkCtx.beginPath(); sparkCtx.moveTo(0, y(0)); sparkCtx.lineTo(w, y(0)); sparkCtx.stroke();
    sparkCtx.strokeStyle = 'rgba(157,154,216,.55)'; sparkCtx.setLineDash([4, 5]);
    sparkCtx.beginPath(); sparkCtx.moveTo(0, y(BUDGET_MS)); sparkCtx.lineTo(w, y(BUDGET_MS)); sparkCtx.stroke();
    sparkCtx.setLineDash([]);
    // Axis labels sit in the gutter beside the canvas; place them on the rules.
    axisBudget.textContent = BUDGET_MS.toFixed(1);
    axisBudget.style.top = (y(BUDGET_MS) / h * 100).toFixed(2) + '%';
    axisZero.style.top = (y(0) / h * 100).toFixed(2) + '%';

    if (n > 1) {
      sparkCtx.beginPath();
      sparkCtx.moveTo(x(0), plotBottom);
      for (let i = 0; i < n; i++) sparkCtx.lineTo(x(i), y(this.history[i]));
      sparkCtx.lineTo(x(n - 1), plotBottom);
      sparkCtx.closePath();
      const grad = sparkCtx.createLinearGradient(0, SPARK_PAD_TOP, 0, plotBottom);
      grad.addColorStop(0, 'rgba(226,162,78,.20)');
      grad.addColorStop(1, 'rgba(226,162,78,0)');
      sparkCtx.fillStyle = grad; sparkCtx.fill();

      sparkCtx.lineWidth = 1.5;
      for (let i = 1; i < n; i++) {
        const a = this.history[i - 1], b = this.history[i];
        sparkCtx.strokeStyle = (a > BUDGET_MS || b > BUDGET_MS) ? '#c2603f' : '#e2a24e';
        sparkCtx.beginPath(); sparkCtx.moveTo(x(i - 1), y(a)); sparkCtx.lineTo(x(i), y(b)); sparkCtx.stroke();
      }
      const lastOver = this.history[n - 1] > BUDGET_MS;
      sparkCtx.fillStyle = lastOver ? '#c2603f' : '#f2cb8e';
      sparkCtx.beginPath(); sparkCtx.arc(x(n - 1), y(this.history[n - 1]), 2.4, 0, Math.PI * 2); sparkCtx.fill();
    }
    $('sparkCaption').textContent = n < 2
      ? 'COLLECTING…'
      : n + ' FRAMES · PEAK ' + (sampleMax > 0 ? fmtMs(sampleMax) : 'BELOW CLOCK QUANTUM');
  }
};
