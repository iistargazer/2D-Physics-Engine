// ============================================================================
// a11y.cjs - contrast and colour-redundancy check for the presentation layer.
//
//   node build/browser-check/a11y.cjs            (needs the page on :8777)
//
// Measures the ratio every rendered text/graphic pair actually has, at the size
// and weight the element really computes, against the surface it is really
// painted on, and fails when a pair drops below WCAG AA (4.5:1 for normal text,
// 3:1 for large text and for graphics that carry meaning).
//
// Two properties make this a regression guard rather than a snapshot:
//   * every foreground is read from the live element, and the token it is
//     declared to be is asserted against it, so editing a token re-measures;
//   * every background literal must still appear in the served source that
//     paints it (showcase.css / core.js / render.js), so editing a gradient
//     cannot leave the check measuring a surface that no longer exists.
//
// Alpha layers are composited bottom-up. A background under text is taken at the
// worst (lightest) end of its gradient, because this page is light-on-dark.
// ============================================================================
const { chromium } = require('playwright');
const assert = require('node:assert/strict');

const PORT = Number(process.env.PORT) > 1 ? process.env.PORT : '8777';
const URL = `http://127.0.0.1:${PORT}/index.html`;

// fg: token name, or {token, alpha} for a colour drawn through an opacity.
// bg: layers painted bottom-up; a token name, a literal hex, or {hex|token, alpha}.
const G = ['--ground'], P = ['--panel'], NAV = ['--nav'], TILE = ['#171b2e'], INSET = ['--inset'];
const PAIRS = [
  // --- reading text -------------------------------------------------------
  {label: '.lede', sel: '.lede', fg: '--ink-2', bg: G},
  {label: '.scene-description', sel: '.scene-description', fg: '--ink-2', bg: P},
  {label: 'field note prose', sel: '.field-notes section p:not(.eyebrow)', fg: '--ink-2', bg: P},
  {label: '.meter-foot', sel: '.meter-foot', fg: '--ink-3', bg: P},
  {label: 'footer span', sel: 'footer span', fg: '--ink-3', bg: G},
  {label: 'engineering prose', sel: '.engineering article p', fg: '--ink-2', bg: G},
  {label: 'engineering heading h3', sel: '.engineering h3', fg: '--ink', bg: G},
  // --- label tier ---------------------------------------------------------
  {label: 'intro eyebrow', sel: '.intro .eyebrow', fg: '--ink-4', bg: G},
  {label: 'build label', sel: '.build-label', fg: '--ink-3', bg: G},
  {label: 'nav-foot', sel: '.nav-foot', fg: '--ink-4', bg: NAV},
  {label: 'nav item', sel: '#scenes button', fg: '--ink-2', bg: NAV},
  {label: 'nav item number', sel: '#scenes button .number', fg: '--ink-4', bg: NAV},
  {label: 'nav selected (label)', sel: '#scenes button.on', fg: '--ink', bg: ['--nav', {token: '--amber', alpha: .09}]},
  {label: 'nav selected (number)', sel: '#scenes button.on .number', fg: '--amber', bg: ['--nav', {token: '--amber', alpha: .09}]},
  {label: 'section label', sel: '#dashHead', fg: '--ink-2', bg: P},
  {label: 'section label meta', sel: '#dashHead span', fg: '--ink-4', bg: P},
  {label: 'sheet number', sel: '.dashboard .sheet-num', fg: '--amber-2', bg: P},
  {label: 'tune label', sel: '.tune>label', fg: '--ink-3', bg: P},
  {label: 'status stamp', sel: '.status', fg: '--ink-2', bg: P},
  {label: 'field note eyebrow', sel: '.field-notes .eyebrow', fg: '--ink-3', bg: P},
  // --- ledger and meters --------------------------------------------------
  {label: 'tile label', sel: '.telemetry>div>span', fg: '--ink-3', bg: TILE},
  {label: 'tile value', sel: '.telemetry strong', fg: '--ink', bg: TILE},
  {label: 'tile unit', sel: '.telemetry strong i', fg: '--ink-4', bg: TILE},
  {label: 'meter head', sel: '.meter-head', fg: '--ink-3', bg: P},
  {label: 'verdict', sel: '#solverMode', fg: '--ink-3', bg: P},
  {label: 'meter scale', sel: '.meter-scale', fg: '--ink-4', bg: P},
  {label: 'phase label', sel: '.phase>span', fg: '--ink-3', bg: P},
  {label: 'phase value', sel: '.phase em', fg: '--ink-2', bg: P},
  {label: 'phase unit', sel: '.phase em i', fg: '--ink-4', bg: P},
  {label: 'tune unit', sel: '.unit', fg: '--ink-4', bg: P},
  {label: 'tune scale', sel: '.tune-scale', fg: '--ink-4', bg: P},
  {label: 'spark axis', sel: '.spark-axis', fg: '--ink-4', bg: INSET},
  {label: 'ruler edge', sel: '.scale-edge.lo', fg: '--ink-4', bg: INSET},
  {label: 'stage camera label', sel: '.stage-label', fg: '--ink-3', bg: ['--stage-top']},
  // --- controls and their states -----------------------------------------
  {label: 'toolbar button', sel: '#restart', fg: '--ink-2', bg: ['--panel-2']},
  {label: 'tuning toggle', sel: '#ngs', fg: '--ink-2', bg: ['--panel-2']},
  {label: 'tuning value', sel: '.tune output', fg: '--ink', bg: P},
  {label: 'ghost button', sel: '.ghost', fg: '--ink-2', bg: ['--ground', {token: '--amber', alpha: .09}]},
  {label: 'kbd hint', sel: 'kbd', fg: {token: '--ink-2', alpha: .78}, bg: ['--panel-2']},
  // --- semantic states (classes forced, so the rule is what is measured) --
  {label: 'pressed control (dark on amber)', sel: '#pause', fg: {hex: '#191004'}, bg: ['--amber']},
  {label: 'state stamp (live)', sel: '.state-pill', fg: '--amber-3', bg: ['--panel', {token: '--amber', alpha: .08}]},
  {label: 'state stamp (idle)', sel: '.state-pill.idle', fg: '--ink-3', bg: P},
  {label: 'state stamp (strain)', sel: '.state-pill.strain', fg: '--rust-2', bg: ['--panel', {token: '--rust', alpha: .10}]},
  {label: 'verdict good', sel: '.verdict.good', fg: '--amber-3', bg: P},
  {label: 'verdict over', sel: '.verdict.over', fg: '--rust-2', bg: P},
  {label: 'first-run tag', sel: '.first-run-tag', fg: '--amber-3', bg: ['--ground', {token: '--amber', alpha: .09}]},
  {label: 'first-run prose', sel: '.first-run-steps', fg: '--ink-2', bg: ['--ground', {token: '--amber', alpha: .09}]},
  {label: 'recovery heading', sel: '.runtime-error strong', fg: '--rust-2', bg: ['--panel', {token: '--rust', alpha: .07}]},
  {label: 'recovery prose', sel: '.runtime-error p', fg: '--ink-2', bg: ['--panel', {token: '--rust', alpha: .07}]},
  {label: 'recovery code chip', sel: '.runtime-error code', fg: '--amber-3', bg: INSET},
  {label: 'instructions summary', sel: 'summary', fg: '--ink-2', bg: P},
  {label: 'instructions prose', sel: '.instructions', fg: '--ink-3', bg: P},
  {label: 'legend text (names the hues)', sel: '.meter-scale.keys', fg: '--ink-3', bg: P},
  {label: 'article tag', sel: '.art-tag', fg: '--amber-2', bg: G},
  {label: 'pointer read-out', sel: '.tools>span', fg: '--ink-3', bg: P},
  {label: 'ruler step', sel: '.scale-step', fg: '--ink-4', bg: INSET},
  {label: 'budget mark caption', sel: '.budget-mark', prop: 'color', pseudo: '::after', fg: '--ink-4', bg: P}
];

// --- non-text graphics that carry meaning (3:1) -----------------------------
const GRAPHICS = [
  {label: 'awake legend swatch', sel: '.key.awake', prop: 'backgroundColor', fg: '--amber', bg: P},
  {label: 'asleep legend swatch', sel: '.key.asleep', prop: 'backgroundColor', fg: '--lav', bg: P},
  {label: 'static legend swatch', sel: '.key.static', prop: 'backgroundColor', fg: '--teal', bg: P},
  {label: 'static composition segment', sel: '#segStatic', prop: 'backgroundColor', fg: '--teal', bg: INSET},
  {label: 'awake composition segment', sel: '#segAwake', prop: 'backgroundImage', fg: '--amber', bg: INSET},
  {label: 'budget fill', sel: '#budgetFill', prop: 'backgroundImage', fg: '--amber', bg: INSET},
  {label: 'over-budget fill', sel: '#budgetFill', prop: 'backgroundImage', fg: '--rust', bg: INSET},
  {label: 'budget 60 Hz mark', sel: '.budget-mark', prop: 'backgroundColor', fg: '--ink-3', bg: INSET},
  {label: 'phase bar fill', sel: '.phase-bar b', prop: 'backgroundImage', fg: '--amber-2', bg: INSET},
  {label: 'ccd bar fill', sel: '.phase[data-stage="ccd"] .phase-bar b', prop: 'backgroundImage', fg: '--lav-2', bg: INSET},
  {label: 'tile state rule (awake)', sel: '[data-tile="awake"]', prop: 'boxShadow', fg: '--amber', bg: TILE},
  {label: 'tile state rule (asleep)', sel: '[data-tile="asleep"]', prop: 'boxShadow', fg: '--lav', bg: TILE},
  {label: 'tile state rule (static)', sel: '[data-tile="static"]', prop: 'boxShadow', fg: '--teal', bg: TILE},
  {label: 'tile state rule (step)', sel: '[data-tile="step"]', prop: 'boxShadow', fg: '--amber', bg: TILE},
  {label: 'tile state rule (bodies)', sel: '[data-tile="bodies"]', prop: 'boxShadow', fg: '--ink-2', bg: TILE},
  {label: 'tile state rule (contacts)', sel: '[data-tile="contacts"]', prop: 'boxShadow', fg: '--amber-2', bg: TILE},
  {label: 'asleep composition segment', sel: '#segAsleep', prop: 'backgroundImage', fg: '--lav', bg: INSET}
];
// Stage content, measured from the renderer's own constants and literals.
const STAGE = [
  {label: 'stage body: static', hexIn: ['core.js'], fg: {slot: 0}, bg: ['--stage-top']},
  {label: 'stage body: awake', hexIn: ['core.js'], fg: {slot: 1}, bg: ['--stage-top']},
  {label: 'stage body: asleep', hexIn: ['core.js'], fg: {slot: 2}, bg: ['--stage-top']},
  {label: 'stage contact dot', hexIn: ['render.js'], fg: {hex: '#f2cb8e'}, bg: ['--stage-top']},
  {label: 'stage impulse line', hexIn: ['render.js'], fg: {hex: '#e08b62'}, bg: ['--stage-top']},
  {label: 'stage constraint guide', hexIn: ['render.js'], fg: {hex: '#a6a3b8', alpha: 170 / 255}, bg: ['--stage-top']},
  {label: 'stage body outline', hexIn: ['render.js'], fg: {hex: '#efe6d5', alpha: 128 / 255}, bg: ['--stage-top']}
];
// Reported, never asserted: structure that carries no information on its own.
const GRAPHICS_MORE = [
  {label: 'ruler ticks', sel: '.scale-ticks', prop: 'backgroundImage', fg: '--rule-strong', bg: INSET},
  {label: 'ruler fifth-step bars', sel: '.scale-bars', prop: 'backgroundImage', fg: '--rule-strong', bg: INSET}
];
const REPORTED = [
  {label: 'stage grid (decorative)', hexIn: ['render.js'], fg: {hex: '#262f4d'}, bg: ['--stage-mid']},
  {label: 'hairline rule (decorative)', sel: '.workspace', prop: 'borderTopColor', fg: '--rule', bg: G},
  // The slider thumb is identified by shape and by its own dark outline, not by
  // hue against the track. Its colour lives on ::-webkit-slider-thumb, which no
  // element reports, so this one is declared-and-reported rather than asserted.
  {label: 'slider thumb vs track (declared)', fg: '--amber', bg: ['--rule']}
].map(r => ({...r, kind: 'report'}));

// --- contrast maths ---------------------------------------------------------
function parse(value) {
  if (value.startsWith('#')) {
    const h = value.length === 4 ? value.slice(1).split('').map(c => c + c).join('') : value.slice(1);
    return [0, 2, 4].map(i => parseInt(h.slice(i, i + 2), 16) / 255);
  }
  const nums = value.match(/[\d.]+/g).map(Number);
  return [nums[0] / 255, nums[1] / 255, nums[2] / 255];
}
const over = (top, alpha, bottom) => top.map((c, i) => c * alpha + bottom[i] * (1 - alpha));
function luminance([r, g, b]) {
  const f = c => c <= 0.03928 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4);
  return 0.2126 * f(r) + 0.7152 * f(g) + 0.0722 * f(b);
}
function ratio(a, b) {
  const la = luminance(a), lb = luminance(b);
  return (Math.max(la, lb) + 0.05) / (Math.min(la, lb) + 0.05);
}
const rgbString = ([r, g, b]) => `rgb(${Math.round(r * 255)}, ${Math.round(g * 255)}, ${Math.round(b * 255)})`;
// getComputedStyle serialises a box-shadow colour as rgba(r, g, b, 1) and a
// gradient stop as rgb(r, g, b), so the "really paints this token" check accepts
// both spellings.
const paints = (text, fg) => {
  const opaque = rgbString(fg);
  return text.includes(opaque) || text.includes(opaque.replace('rgb(', 'rgba(').replace(')', ', 1)'));
};
const ALL = [...PAIRS.map(p => ({...p, kind: 'text'})), ...GRAPHICS, ...GRAPHICS_MORE, ...STAGE, ...REPORTED];

(async () => {
  const browser = await chromium.launch({
    executablePath: 'C:/Program Files/Google/Chrome/Application/chrome.exe', headless: true
  });
  const context = await browser.newContext({ viewport: { width: 1440, height: 1180 } });
  const page = await context.newPage();
  const errors = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.goto(URL);
  await page.waitForFunction(() => runtimeReady && Module._frame_size() > 5);

  // Force the state classes and open the panels, so each rule is measured in the
  // state it exists for rather than only in the state the page happens to load.
  await page.evaluate(() => {
    selectScene(3);
    Module._set_profile(1);
    Instrument.sync(false);
    document.querySelector('#phasePanel').classList.remove('off');
  });
  await page.waitForTimeout(400);

  const live = await page.evaluate((specs) => {
    const root = getComputedStyle(document.documentElement);
    // Forced and read in this one synchronous pass: the frame loop rewrites these
    // classes every tick, and the controls transition their colour, so a state
    // forced in an earlier pass (or measured after an await) would report either
    // the live state or a half-finished transition rather than the rule.
    const noMotion = document.createElement('style');
    noMotion.textContent = '*{transition:none !important;animation:none !important}';
    document.head.appendChild(noMotion);
    document.querySelector('#runtimeError').hidden = false;
    document.querySelector('#budgetVerdict').classList.remove('over');
    document.querySelector('#budgetVerdict').classList.add('good');
    document.querySelector('#stageTotal').classList.remove('good');
    document.querySelector('#stageTotal').classList.add('over');
    document.querySelector('#statePill').classList.add('strain');
    document.querySelector('#statePill').classList.remove('idle');
    document.querySelector('#pause').classList.add('on');
    document.querySelector('#restart').classList.remove('on');
    document.querySelector('#ngs').classList.remove('on');
    document.querySelectorAll('.instructions').forEach(d => { d.open = true; });
    const props = [...new Set(specs.map(s => s.prop).filter(Boolean))];
    const read = spec => {
      const el = spec.sel ? document.querySelector(spec.sel) : null;
      if (spec.sel && !el) return null;
      const cs = el ? getComputedStyle(el, spec.pseudo || null) : null;
      const painted = {};
      if (cs) for (const prop of props) painted[prop] = cs[prop] || '';
      return {color: cs ? cs.color : '', painted,
              size: cs ? parseFloat(cs.fontSize) : 0, weight: cs ? cs.fontWeight : '0'};
    };
    const wants = specs.flatMap(s =>
      [typeof s.fg === 'string' ? s.fg : s.fg && s.fg.token,
       ...(s.bg || []).map(l => typeof l === 'string' ? l : l.token)]);
    const names = [...new Set(wants.filter(n => n && n.indexOf('--') === 0))];
    return {
      tokens: Object.fromEntries(names.map(n => [n, root.getPropertyValue(n).trim()])),
      colors: typeof COLORS !== 'undefined' ? COLORS : null,
      elements: Object.fromEntries(specs.filter(s => s.sel).map(s => [s.label, read(s)])),
      legend: (document.querySelector('.meter-scale.keys') || {}).textContent || '',
      pillText: (document.querySelector('#stateText') || {}).textContent || '',
      tileLabels: [...document.querySelectorAll('.telemetry>div>span')].map(s => s.textContent.trim()),
      phaseLabels: [...document.querySelectorAll('.phase>span')].map(s => s.textContent.trim()),
      rulerHidden: document.querySelector('.stage-scale') ? document.querySelector('.stage-scale').getAttribute('aria-hidden') : null
    };
  }, ALL.map(p => ({label: p.label, sel: p.sel, prop: p.prop, pseudo: p.pseudo, fg: p.fg, bg: p.bg})));

  // Two elements each carry mutually exclusive states that the main pass can only
  // hold one of at a time: the status stamp (strain was measured there) and the
  // budget fill (the over-budget pattern). Read the remaining ones here; the
  // no-motion rule from the main pass still applies, so these are exact.
  live.extraStates = await page.evaluate(() => {
    const pill = document.querySelector('#statePill');
    pill.classList.remove('strain', 'idle');
    const plain = getComputedStyle(pill).color;
    pill.classList.add('idle');
    const idle = getComputedStyle(pill).color;
    const bar = document.querySelector('#budgetBar');
    bar.classList.add('over');
    const overFill = getComputedStyle(document.querySelector('#budgetFill')).backgroundImage;
    bar.classList.remove('over');
    return {plain, idle, overFill};
  });

  const sources = {};
  for (const f of ['showcase.css', 'core.js', 'render.js', 'instrument.js']) {
    sources[f] = await page.evaluate(f => fetch(f).then(r => r.text()), f);
  }

  // The three body colours are read from the served core.js text, not from a page
  // global, so they come from the same file a maintainer would edit.
  const colorsInSource = (sources['core.js'].match(/COLORS\s*=\s*\[([^\]]*)\]/) || ['', ''])[1]
    .split(',')
    .map(c => c.replace(/['"\s]/g, ''))
    .filter(Boolean);
  assert.equal(colorsInSource.length, 3, 'core.js still declares exactly three body colours');

  let failures = 0, checked = 0, missing = 0;
  const rows = [];
  for (const p of ALL) {
    let el = p.sel ? live.elements[p.label] : {color: '', painted: {}, size: 0, weight: '0'};
    if (p.label === 'state stamp (idle)') el = {...el, color: live.extraStates.idle};
    if (p.label === 'state stamp (live)') el = {...el, color: live.extraStates.plain};
    if (p.label === 'over-budget fill') el = {...el, painted: {...el.painted, backgroundImage: live.extraStates.overFill}};
    if (!el) { console.log(`MISSING  ${p.label} (${p.sel}) is not on the page`); failures++; missing++; continue; }

    const fgSpec = typeof p.fg === 'string' ? {token: p.fg} : p.fg;
    let fgHex, fg;
    if (fgSpec.slot !== undefined) {
      fgHex = colorsInSource[fgSpec.slot];
      fg = parse(fgHex);
      assert.ok(sources['core.js'].toLowerCase().includes(fgHex.toLowerCase()), `${p.label}: ${fgHex} still in core.js`);
    } else if (fgSpec.token) {
      fgHex = live.tokens[fgSpec.token];
      assert.ok(fgHex, `${p.label}: token ${fgSpec.token} resolves`);
      fg = parse(fgHex);
    } else {
      fgHex = fgSpec.hex; fg = parse(fgHex);
      if (p.hexIn) assert.ok(sources[p.hexIn[0]].toLowerCase().includes(fgHex.toLowerCase()), `${p.label}: ${fgHex} still in ${p.hexIn[0]}`);
    }

    let bg = [0, 0, 0];
    for (const layer of p.bg) {
      // A layer is a token name ('--panel'), a literal hex ('#171b2e'), or a tint
      // of either ({token|hex, alpha}).
      const spec = typeof layer === 'string' ? {name: layer} : layer;
      const isToken = spec.name ? spec.name.indexOf('--') === 0 : !!spec.token;
      const hex = isToken ? live.tokens[spec.token || spec.name] : (spec.hex || spec.name || live.tokens[spec.token]);
      assert.ok(hex, `${p.label}: background ${JSON.stringify(layer)} resolves`);
      bg = over(parse(hex), spec.alpha === undefined ? 1 : spec.alpha, bg);
      if (!isToken && !p.hexIn) assert.ok(sources['showcase.css'].toLowerCase().includes(hex.toLowerCase()), `${p.label}: surface ${hex} still in showcase.css`);
    }

    // The declaration must be what the element actually paints.
    if (p.sel && (!p.prop || p.prop === 'color')) assert.equal(el.color, rgbString(fg), `${p.label}: computed color equals ${fgHex}`);
    else if (p.sel) assert.ok(paints(el.painted[p.prop], fg), `${p.label}: ${p.sel} paints ${fgHex} in ${p.prop}`);
    if (fgSpec.alpha) fg = over(fg, fgSpec.alpha, bg);   // kbd is drawn through opacity

    const r = ratio(fg, bg);
    const large = el.size >= 24 || (el.size >= 18.66 && Number(el.weight) >= 700);
    const need = p.kind === 'text' ? (large ? 3 : 4.5) : 3;
    const verdict = r >= need ? 'PASS' : (p.kind === 'report' ? 'note' : 'FAIL');
    if (verdict === 'FAIL') failures++;
    if (p.kind !== 'report') checked++;
    rows.push({label: p.label, px: el.size, weight: el.weight, ratio: +r.toFixed(2), need, verdict});
  }

  // --- no meaning may survive as colour alone -------------------------------
  const note = [];
  // Collected, not asserted on the spot, so one run reports every failure and
  // still prints the measured table.
  const must = (value, message) => {
    if (!value) failures++;
    note.push((value ? 'PASS ' : 'FAIL ') + message);
  };
  must(/AWAKE/.test(live.legend) && /ASLEEP/.test(live.legend) && /STATIC/.test(live.legend),
    'every composition hue is named in text next to its bar');
  must(live.tileLabels.length === 6 && live.tileLabels.every(t => t.length > 2),
    'every telemetry tile is labelled in text, so its state rule is never the only cue');
  must(live.phaseLabels.length === 7,
    'every stage row is labelled in text, including the one drawn in the dormant hue');
  must(/repeating-linear-gradient/.test(live.extraStates.overFill),
    'the over-budget fill carries a hatch as well as a hue');
  must(sources['render.js'].includes('DASH_ASLEEP'),
    'the renderer outlines dormant bodies with a dash, so awake/asleep is not hue-only in the stage');
  must(live.rulerHidden === 'true',
    'the scale ruler is marked decorative; the view span is stated in text by the camera label');
  const hue = hex => {
    const [r, g, b] = parse(hex), max = Math.max(r, g, b), min = Math.min(r, g, b), d = max - min;
    if (!d) return 0;
    const h = max === r ? ((g - b) / d + 6) % 6 : max === g ? (b - r) / d + 2 : (r - g) / d + 4;
    return h * 60;
  };
  const stageHues = ['--teal', '--amber', '--lav'];
  must(stageHues.every((t, i) => live.tokens[t] &&
      Math.abs(((hue(live.tokens[t]) - hue(colorsInSource[i]) + 540) % 360) - 180) < 20),
    'the stage draws static/awake/dormant bodies in the hues the legend names, staged darker: ' +
    colorsInSource.join(' ') + ' vs ' + stageHues.map(t => live.tokens[t]).join(' '));
  must(live.pillText.trim().length > 0,
    'the live state stamp prints its state as a word, not as a hue alone: ' + JSON.stringify(live.pillText.trim()));
  must(errors.length === 0, 'no browser runtime errors: ' + JSON.stringify(errors));

  // Pairs and assertions are counted apart: a reader who sees "19 below AA" should
  // find nineteen failing rows, not eighteen and a broken invariant.
  const pairFails = rows.filter(r => r.verdict === 'FAIL').length + missing;
  const assertionFails = failures - pairFails;
  const reported = rows.filter(r => r.verdict === 'note');
  if (reported.length) console.log('reported, not asserted (' + reported.length + '): ' + reported.map(r => r.label).join(', ') + String.fromCharCode(10));
  const w = Math.max(...rows.map(r => r.label.length));
  console.log('pair'.padEnd(w) + '   px  weight   ratio  need  result');
  for (const r of rows) {
    console.log(`${r.label.padEnd(w)}  ${String(r.px).padStart(2)}  ${String(r.weight).padStart(6)}  ${String(r.ratio).padStart(6)}  ${String(r.need).padStart(4)}  ${r.verdict}`);
  }
  console.log('\n' + note.join('\n'));
  console.log(`\nchecked ${checked} pairs, ${pairFails} below AA` + (assertionFails ? `, ${assertionFails} assertion(s) failed` : ', every assertion above passed'));
  await browser.close();
  if (failures) { console.error(`${pairFails} pairs below AA, ${assertionFails} assertion(s) failed`); process.exit(1); }
  console.log('ALL CONTRAST CHECKS PASSED');
})().catch(e => { console.error(e); process.exit(1); });
