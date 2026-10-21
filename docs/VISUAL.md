# The visual identity of the mechanics lab

The browser page is the project's showcase, so its presentation layer is part of the
deliverable rather than packaging around it. This note records what the second pass changed,
what the visual system is now, and which of those claims are backed by a receipt.

Scope: [web/shell.html](../web/shell.html) (structure), [web/showcase.css](../web/showcase.css)
(type, palette, grid, motion), [web/instrument.js](../web/instrument.js) (what each read-out
says), and eight colour constants in [web/core.js](../web/core.js) /
[web/render.js](../web/render.js) that paint the stage (three body colours and five stage marks). The engine, the frame-buffer layout and
the stats buffer are untouched: the renderer only reads the same two additive buffers it always
read. No capability was added — nothing new can be measured, toggled or spawned that could not
be measured, toggled or spawned before.

## 1. What the first draft got wrong

The first attempt was a working instrument panel with a template's surface. Four specific
problems, in the order they hurt:

1. **One hue.** Background, panels, borders, accent, bars and the stage were all samples of a
   single green ramp. Colour carried no information, so every panel had to be read in full.
2. **One value.** Nav, experiment plate and stage sat within a few percent of each other in
   brightness, so the page read as a flat sheet with no depth or hierarchy.
3. **One voice.** Mono was used for labels, values, prose captions and controls alike. With no
   change of face, hierarchy rested entirely on font size.
4. **One rhythm.** Every section was a full-width band with the same top hairline and the same
   spacing, so the page was a stack of equally weighted rows — and 8px radii, a pill-shaped
   status chip and a glow under the brand mark are the three most recognisable marks of a
   generated dark dashboard.

## 2. The identity: a twilight palette, four hues and no more

The palette comes from the two references attached to the request — Parrish's ultramarine sky
with amber foliage over a parchment figure, and the violet-mountain / ochre-hill / teal-water
landscape — combined with the discipline of <https://keykyrios.github.io/>: a warm limited
palette, hairline rules, squared corners, widely tracked small mono labels and generous space
between major sections. No element of that site is reproduced; its palette and rhythm are the
reference, not its content.

| Token | Value | Meaning |
|---|---|---|
| `--ground` / `--panel` / `--panel-2` | `#0c0e17` / `#101322` / `#151930` | ultramarine ground, plate, raised tile |
| `--stage-top` → `--stage-low` | `#161c33` → `#090b16` | the world view is a lit sky above a dark floor |
| `--rule` / `--rule-soft` / `--rule-strong` | `#252b45` / `#1a1f33` / `#6a7095` | hairline and interior hairline (separators, deliberately faint), and the one that carries information — the metre ruler's ticks and bars, 4.1 : 1 |
| `--ink` … `--ink-4` | `#ece5d5` → `#8f8ca0` | parchment reading text: four tiers, every one of them above AA (§7) |
| `--amber` / `--amber-2` / `--amber-3` | `#e2a24e` / `#b57d31` / `#f2cb8e` | **warm — energy**: awake bodies, solver cost, the live state |
| `--lav` / `--lav-2` | `#9d9ad8` / `#6f6da8` | **cool — dormant**: sleeping bodies, the CCD prototype path, the budget rule |
| `--teal` | `#5f9ea6` | **cold — static**: frozen geometry |
| `--rust` / `--rust-2` | `#c2603f` / `#e08b62` | **alert**: a step over the frame budget, and nothing else |

The rule that makes it a system rather than a palette: **a hue means one thing**. Awake is warm
everywhere it appears — the body in the stage, the AWAKE tile's rule, its swatch in the
composition legend, its bar in the breakdown. Sleeping is lavender in all four places. Static is
teal in all four. Rust never means "fast", "important" or "accent"; it means over budget, which
is why the CCD stage stopped being drawn in it (CCD is the prototype path, so it is drawn from
the dormant hue).

The stage's body colours are the dashboard's hues, not a second scheme:

| Engine state | Colour | Shared with |
|---|---|---|
| 0 static | `#55838c` teal, staged | the STATIC tile's rule and legend swatch — the same hue, darker here |
| 1 awake | `#dba45c` amber | the AWAKE tile, the budget fill, the history line |
| 2 asleep | `#7d7aa8` lavender | the ASLEEP tile and the drowsy end of the composition bar |
| CCD bullet (scene 8) | `#e08b62` rust | over-budget only — the one deliberate exception, documented in [SHOWCASE](SHOWCASE.md) |

## 3. Three type voices

| Voice | Stack | Used for |
|---|---|---|
| Display | `Palatino, "Palatino Linotype", Georgia, serif` | `h1`, scene titles, section headings, the wordmark |
| Instrument | `ui-monospace, "Cascadia Mono", Menlo, Consolas, monospace` | every label, number, unit, control and status token |
| Prose | `system-ui, -apple-system, "Segoe UI", Roboto` | lede, scene descriptions, meter footnotes, field notes |

Hierarchy now moves on three axes at once instead of one. A label is 9–10px mono with
`.14em–.2em` tracking and the faint tier; a value is 24–27px mono with tabular figures and the
brightest ink; a unit is 8–11px mono in the faintest tier, set beside the number rather than
inside it. Sentence-case serif headings sit against uppercase mono labels, which is what makes
the section numbers read as an index rather than as more chrome.

## 4. Layout: numbered plates, a ruler and a deck

- **Section numbers run 01–05 without a gap** — `01 EXPERIMENTS`, `02 TELEMETRY`,
  `03 STEP BREAKDOWN`, `04 TUNING`, `05 UNDER THE SURFACE` — each in an amber gutter cell at the
  left of its own hairline head. The first draft numbered the nav 01 and the next panel 03.
- **Telemetry is a ledger, not a card grid.** Six tiles share hairline rules with no gaps, and
  each tile carries a 2px left rule in its own hue, so the block doubles as the colour key.
- **The stage has a metre ruler.** `instrument.js` derives the on-screen size of one metre from
  the same camera width the pointer inverse maps through, snaps the tick step to
  `0.5/1/2/5/10/20/50/100 m` so ticks never collapse into a haze, and writes the two edge
  labels as the world coordinates the view actually spans (`−27.50 m … 23.50 m` for the
  pyramid). Every fifth step is a full-height bar, so the span is countable.
- **The cost history is a calibrated plot.** The 60 Hz rule is labelled from the same two pad
  constants the series is drawn with, so a gridline and its label cannot drift; the axis labels
  are DOM text in a gutter, which stays legible at any width.
- **The breakdown and the tuning panel sit side by side above 1180px** instead of forming two
  more full-width bands, with a shared top rule and a vertical hairline between them. Below
  that width the deck is one column again.
- **Squared, not rounded.** Every surface is 0–2px radius; the status chip became a squared
  stamp; sliders became faders — an inset groove with a squared handle — with the input's own
  `min`/`max` printed at the ends of the track.

### Cut rather than restyled

- The nav hint ("Choose a system to investigate") — a sentence that told the reader nothing the
  list did not.
- The second "steps simulated" phrasing next to the composition bar, which duplicated the step
  counter that already sits in the simulated-time token.
- The redundant `<b>` budget read-out inside the gauge foot, which repeated the value already
  printed in the step tile.
- The lime glow pulse on the brand mark, and the hover lift on every control — motion that
  acknowledged the pointer rather than the engine.

## 5. Motion

Only state changes move: the budget fill and composition segments animate when a measured value
changes, a scene change fires one wipe across the stage, the sweep is a slow light band over a
sky that is already a gradient, and the live dot pulses while the runtime is healthy. Every one
of those is asserted off under `prefers-reduced-motion: reduce`, which is a real receipt rather
than a promise.

## 6. Evidence

Captured by `build/browser-check/evidence.cjs` (disposable, like its sibling suites) against the
build served on `127.0.0.1:8777`; the `before` set was captured the same way from the previous
revision's own build output, served separately, so both sides are the same views at the same
scene and window sizes.

| View | Before | After |
|---|---|---|
| Desktop 1440 (wrecking pyramid, profile on, paused) | [before-desktop.jpg](screenshots/before-desktop.jpg) | [after-desktop.jpg](screenshots/after-desktop.jpg) |
| Tablet 1024 | [before-tablet.jpg](screenshots/before-tablet.jpg) | [after-tablet.jpg](screenshots/after-tablet.jpg) |
| Narrow 390 | [before-narrow.jpg](screenshots/before-narrow.jpg) | [after-narrow.jpg](screenshots/after-narrow.jpg) |
| First load, cradle running, first-run strip | — | [after-entry.jpg](screenshots/after-entry.jpg) |
| Telemetry block (stress field, 5,000 awake, over budget) | [before-dashboard.png](screenshots/before-dashboard.png) | [after-dashboard.png](screenshots/after-dashboard.png) |
| Step breakdown (same loaded field) | [before-breakdown.png](screenshots/before-breakdown.png) | [after-breakdown.png](screenshots/after-breakdown.png) |
| Same field settled (5,000 asleep, inside budget) | — | [after-settled.png](screenshots/after-settled.png) |
| Tuning faders, as the pyramid seeds them | — | [after-tuning.png](screenshots/after-tuning.png) |

The empty stage in a `before` crop is the stress field's own framing (5,000 small boxes across a
wide, shallow camera), not a rendering failure; [SCENES](SCENES.md) owns that experiment.

### Behaviour receipts on this build

The three existing suites were **not modified** for this pass and re-run against the rebuilt
page, each as a plain command whose exit status is the result:

| Suite | Assertions | Receipt |
|---|---:|---|
| `check.cjs` — interaction, mapping, framing, all ten scenes, widths 480–1500, DPR 1/2 | 221 | [presentation-check.log](../build/browser-check/presentation-check.log) |
| `dashboard.cjs` — tiles, gauges, stage split, tuning, recovery, reduced motion | 87 | [presentation-dashboard.log](../build/browser-check/presentation-dashboard.log) |
| `framing.cjs` — every scene framed at 1280 and 390 | 50 | [presentation-framing.log](../build/browser-check/presentation-framing.log) |

All three exited 0 with an **empty page-error list**, and the two counts that matter behaviourally
(the simulated step counter equals the driver's step count, and the gauge equals the step it was
given) are asserted inside them. `web/build.sh` was re-run so `build/web/index.html` is generated
from the current shell, and all six copied assets are byte-identical to their `web/` sources.

### Limits

- **Appearance is asserted where it can be measured.** [a11y.cjs](../build/browser-check/a11y.cjs)
  fails on a contrast or colour-redundancy regression (§7), and it is the one script under
  `build/browser-check/` that is versioned rather than disposable. Everything else about the
  look — composition, spacing, texture — is still unasserted: the screenshots above are the only
  record for that, captured by hand at one device pixel ratio on one host.
- The images are JPEG (q82) for the wide views and PNG for the panel crops, to keep the
  repository light; neither is a colour-managed reference.
- The `before` set covers the same three widths and the two panel crops only. The `after` set
  adds the entry state, the settled state and the tuning panel.
- No rendered-frame-rate claim is made anywhere in this pass, and no timing figure moved: the
  palette is not a performance change. Physical touch devices and other browser engines remain
  untested, as [SHOWCASE](SHOWCASE.md) already says.

## 7. Accessibility, measured

The palette was chosen for hierarchy, and that left two things unmeasured: the faintest ink tier
was picked by eye, and much of the meaning was carried by hue. The instrument is
[`build/browser-check/a11y.cjs`](../build/browser-check/a11y.cjs) — one small check, and the only
script under `build/browser-check/` that is versioned, because a palette edit must fail it in a
fresh clone.

It reads every text colour and every meaningful graphic from the live element, composites the
surface it is really painted on (gradients, tints and alpha included), and compares the two
against WCAG AA: 4.5 : 1 for normal text, 3 : 1 for large text and for graphics that carry
meaning. It then asserts that no state is encoded *only* as colour, and that a foreground value
is the token the source still declares, so moving a token re-measures instead of passing.

**Before the repair: 18 of the 79 measured pairs below AA, plus the hue-coherence assertion
failing. After: 0 of 83, with all nine assertions passing** —
[before](../build/browser-check/a11y-before.log), [after](../build/browser-check/a11y-after.log).
Every one of the eighteen contrast failures was fixed by moving the failing tier, never by
lowering the threshold:

| Pair | Before | After | Need |
|---|---:|---:|---:|
| `--ink-4` text: intro eyebrow, nav foot, nav number, section-label meta, meter/phase/tune scales and units, spark axis, ruler edge and step, budget caption | 2.57–2.96 | 5.21–6.01 | 4.5 |
| `--ink-3` text on the tile gradient (tile label) and on the sky (stage camera label) | 4.48 / 4.42 | 6.00 / 5.93 | 4.5 |
| static legend swatch / static composition segment | 2.16 / 2.29 | 6.09 / 6.48 | 3 |
| stage body: static | 2.75 | 4.02 | 3 |

The rule the measurement produced: **AA is a floor for anything that paints text, so hierarchy is
spent above it.** The four ink tiers now sit at roughly 15 : 1, 9 : 1, 6 : 1 and 5 : 1 on the
lightest surface each is used on, and "faint" is reserved for non-text marks, which is the only
place a designer may legitimately go below the floor. Four pairs the first instrument run never
covered were added with the repair and now stand at 4.08 (ruler fifth-step bars), 3.80 (constraint
guide), 4.37 (body outline) and 6.3 (slider thumb against its track, declared — that one lives on
`::-webkit-slider-thumb`, which no element reports).

Two of the defects the measurement found were not contrast failures at all:

- **The static hue disagreed with itself.** Frozen geometry was slate (`#5a6273`) in the stage
  and teal in the tile rule, the legend swatch and the composition segment; the constraint guide
  was drawn in the *dormant* lavender, which the palette reserves for sleeping, at 2.41 : 1
  (arithmetic on the retired literal, not a browser measurement). The stage now draws static as a
  staged teal (`#55838c`) and the guide as a structural ink tone (`#a6a3b8aa`); the wordmark's
  slab follows. The check asserts that correspondence **by hue** (within 20°) and not by value,
  because the stage is allowed to stage the same hue darker than the tile rule that names it.
- **`--rule-strong` had to move to carry the ruler**, which pulled the slider track up under the
  thumb: the track belongs on the hairline tier, so the thumb now reads at 6.3 : 1 against it
  instead of 2.18 : 1.

**Reported, not asserted.** Two marks stay below their threshold and are printed with their
number as `note` rather than failing: the stage grid (1.41 : 1) and the hairline rules
(1.38 : 1). Both are structure that carries no information of its own and both are exempt as
decoration, but they are listed in the log rather than hidden. Everything that paints text is
asserted.

**No state survives as colour alone.** Nine assertions, all passing: the composition hues are
named in the legend beside their bars; all six tiles are labelled in text, so a tile's hue is
never its only label; all seven stage rows carry a label; the over-budget fill is hatched as well
as rust; dormant bodies are outlined with a dash as well as tinted; the ruler is `aria-hidden`
and redundant with the camera label's text span; the stage's three body colours are the legend's
three hues; the live state stamp prints its own state as a word; and the run ends with an empty
page-error list.

The guard is not vacuous: reverting `--ink-4` to its old value and renaming the dash constant in
a scratch copy of the served build makes the same script exit 1 with **13 pairs below AA and the
dash assertion failing** —
[negative control](../build/browser-check/a11y-negative-control.log) — which is the run that
argues a palette edit cannot pass silently. The *before* receipt was written by the previous
revision of the script, whose closing line counted the failed coherence assertion together with
the pairs ("19 below AA" for eighteen failing rows); the script now counts the two apart.

The first run of the instrument also failed nine pairs the design was innocent of, because a
tinted layer was composited at full opacity — a 9 % amber wash measured as solid amber, which is
why `nav selected` read 1.76 : 1 against itself. The before log above is from the repaired
instrument; fixing it and reclassifying three decorative marks moved twelve rows without a token
changing, which is the reason this check reads its surfaces from the live page instead of a
hand-written list.
