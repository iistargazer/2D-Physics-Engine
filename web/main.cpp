// ============================================================================
// WebAssembly entry point.
//
// This is the SAME C++ engine and the SAME scene factories used by the native
// tests and the SFML demo. Only the drawing is delegated to a canvas, and it is
// delegated efficiently: one flat float buffer per frame instead of a JS call per
// shape, so the browser spends its time in the solver rather than in the bridge.
//
// Buffer layout (all floats):
//   [0] bodyCount [1] contactCount [2] stepMs [3] sceneIndex [4] awakeCount
//   then per body:  kind, colorCode, x, y, radiusOrVertexCount,
//                   (polygons) vertexCount * (x, y)
//   then per contact: x, y, normalX, normalY, normalImpulse
// kind: 0 = circle, 1 = polygon.  colorCode: 0 = static, 1 = awake, 2 = asleep.
//
// The frame buffer above is unchanged. The interactive dashboard reads a SECOND,
// fixed-size buffer (`stats_buffer`) that carries per-stage timings and the live
// solver configuration. It is additive: nothing here alters the default PGS path,
// the scene factories or the frame layout the existing browser checks parse.
// ============================================================================
#include "Showcase.hpp"
#include <emscripten.h>
#include <emscripten/html5.h>
#include <cmath>
#include <vector>

using namespace phys;

namespace {
World g_world = makeShowcase(0);
int g_scene = 0;
double g_last = 0, g_accumulator = 0;
double g_slow = 1.0;
bool g_paused = false, g_stepOnce = false, g_debug = false;
real g_lastMs = 0;
std::vector<float> g_buffer;
constexpr int kHeader = 5;

// ---- additive dashboard instrumentation ------------------------------------
// Profiling defaults OFF. Turning it on only wraps existing work in clock reads
// (World::timings); it changes no physics state, so the default solver path and
// every scene result are untouched. Stage costs are EMA-smoothed because the
// browser clock is coarsened to ~100 us and a 6-body step is shorter than that.
bool g_profile = false;
long long g_steps = 0;
real g_stageEma[7] = {};
constexpr int kStats = 13;   // see buildStats() below
std::vector<float> g_stats;

void buildStats() {
    const auto& cfg = g_world.solver.config();
    g_stats.resize(kStats);
    for (int i = 0; i < 7; ++i) g_stats[i] = static_cast<float>(g_stageEma[i]);
    g_stats[7]  = g_profile ? 1.0f : 0.0f;
    g_stats[8]  = static_cast<float>(cfg.iterations);
    g_stats[9]  = cfg.positionCorrection ? 1.0f : 0.0f;
    g_stats[10] = g_world.sleeping ? 1.0f : 0.0f;
    g_stats[11] = static_cast<float>(g_world.gravity.y);
    g_stats[12] = static_cast<float>(g_steps);
}

void beginFrame() {
    g_buffer.clear();
    g_buffer.resize(kHeader, 0.0f);
}

void pushBody(const RigidBody& b) {
    g_buffer.push_back(b.shape.type == ShapeType::Circle ? 0.0f : 1.0f);
    g_buffer.push_back(b.isStatic() ? 0.0f : (b.awake ? 1.0f : 2.0f));
    g_buffer.push_back(static_cast<float>(b.position.x));
    g_buffer.push_back(static_cast<float>(b.position.y));
    if (b.shape.type == ShapeType::Circle) {
        g_buffer.push_back(static_cast<float>(b.shape.radius));
    } else {
        g_buffer.push_back(static_cast<float>(b.shape.verts.size()));
        for (std::size_t i = 0; i < b.shape.verts.size(); ++i) {
            const Vec2 v = b.worldVertex(i);
            g_buffer.push_back(static_cast<float>(v.x));
            g_buffer.push_back(static_cast<float>(v.y));
        }
    }
}

void renderFrame() {
    beginFrame();
    for (const auto& b : g_world.bodies) pushBody(b);
    // Contact markers are only meaningful for awake pairs; frozen contacts are
    // carried over from the cache and would draw stale points.
    int contacts = 0;
    for (const auto& m : g_world.manifolds()) {
        if (!m.bodyA->awake && !m.bodyB->awake) continue;
        for (int i = 0; i < m.count; ++i) {
            const auto& c = m.contactPoints[i];
            g_buffer.push_back(static_cast<float>(c.position.x));
            g_buffer.push_back(static_cast<float>(c.position.y));
            g_buffer.push_back(static_cast<float>(m.normal.x));
            g_buffer.push_back(static_cast<float>(m.normal.y));
            g_buffer.push_back(static_cast<float>(c.accN));
            ++contacts;
        }
    }
    int awake = 0;
    for (const auto& b : g_world.bodies) if (!b.isStatic() && b.awake) ++awake;
    g_buffer[0] = static_cast<float>(g_world.bodies.size());
    g_buffer[1] = static_cast<float>(contacts);
    g_buffer[2] = static_cast<float>(g_lastMs);
    g_buffer[3] = static_cast<float>(g_scene);
    g_buffer[4] = static_cast<float>(awake);
}

// One animation frame. The host shell decides *when* to call this (see
// shell.html): requestAnimationFrame when the browser schedules frames, and a
// setTimeout cadence when it does not. Some headless/background contexts never
// fire requestAnimationFrame at all, and a demo that silently renders zero
// frames in front of an audience is worse than one that ticks from a timer.
void tick() {
    const double now = emscripten_get_now();
    if (g_last == 0) g_last = now;
    double elapsed = (now - g_last) / 1000.0;
    g_last = now;
    if (elapsed > 0.25) elapsed = 0.25;          // clamp a stalled tab
    if (!g_paused) g_accumulator += elapsed * g_slow;
    if (g_stepOnce) { g_accumulator = 1.0 / 60; g_stepOnce = false; }
    while (g_accumulator >= 1.0 / 60) {
        const double start = emscripten_get_now();
        g_world.step(1.0 / 60);
        const double cost = emscripten_get_now() - start;
        // Browsers coarsen the clock to ~100 us, which is longer than a 6-body
        // scene takes to step, so a raw reading rounds to '0.00 ms'. An
        // exponential moving average recovers the true mean step cost.
        g_lastMs = g_lastMs <= 0 ? static_cast<real>(cost)
                                 : static_cast<real>(0.9 * g_lastMs + 0.1 * cost);
        // Same EMA treatment per stage. Only measured when the dashboard asks for
        // it, so the default frame pays nothing extra.
        if (g_profile) {
            const real raw[7] = {
                g_world.timings.broadphase, g_world.timings.narrowphase,
                g_world.timings.islands, g_world.timings.solve,
                g_world.timings.integrate, g_world.timings.ccd,
                g_world.timings.bookkeeping};
            for (int i = 0; i < 7; ++i)
                g_stageEma[i] = g_stageEma[i] <= 0 ? raw[i]
                                                    : static_cast<real>(0.9 * g_stageEma[i] + 0.1 * raw[i]);
        }
        ++g_steps;
        g_accumulator -= 1.0 / 60;
    }
    renderFrame();
    EM_ASM({
        Module.drawFrame(Module.framePtr(), Module.frameCount(), Module.debugOn());
    });
}
} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE void select_scene(int scene) {
    g_scene = scene;
    g_world = makeShowcase(scene);
    g_world.profiling = g_profile;   // the dashboard preference survives a restart
    g_accumulator = 0;
    g_stepOnce = false;
    g_last = 0;
    g_lastMs = 0;
    g_paused = false;
    g_steps = 0;
    for (int i = 0; i < 7; ++i) g_stageEma[i] = 0;
}

EMSCRIPTEN_KEEPALIVE void spawn_box(double x, double y, double angle) {
    auto b = RigidBody::makeBox(0.9, 0.9, 1, {x, y}, angle);
    b.restitution = 0.15;
    g_world.add(b);
}

EMSCRIPTEN_KEEPALIVE void spawn_circle(double x, double y) {
    auto b = RigidBody::makeCircle(0.5, 1, {x, y});
    b.restitution = 0.35;
    g_world.add(b);
}

// Called once per frame by the shell (rAF when available, timer otherwise).
EMSCRIPTEN_KEEPALIVE void tick_js() { tick(); }

EMSCRIPTEN_KEEPALIVE void toggle_pause() { g_paused = !g_paused; }
EMSCRIPTEN_KEEPALIVE void step_once() { g_stepOnce = true; }
EMSCRIPTEN_KEEPALIVE void set_slow(double factor) { g_slow = factor; }
EMSCRIPTEN_KEEPALIVE void toggle_debug() { g_debug = !g_debug; }
EMSCRIPTEN_KEEPALIVE int is_paused() { return g_paused ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE int debug_enabled() { return g_debug ? 1 : 0; }

// Flat per-frame buffer shared with JavaScript (see the layout note above).
EMSCRIPTEN_KEEPALIVE const float* frame_buffer() { return g_buffer.data(); }
EMSCRIPTEN_KEEPALIVE int frame_size() { return static_cast<int>(g_buffer.size()); }

// ---- additive dashboard surface --------------------------------------------
// Fixed-size stats buffer. Layout is documented at kStats in buildStats().
// Refreshed on read rather than on write: the shell can then read the live
// configuration before the first frame has ever been ticked, which is exactly
// when it needs to seed the tuning controls.
EMSCRIPTEN_KEEPALIVE const float* stats_buffer() { buildStats(); return g_stats.data(); }
EMSCRIPTEN_KEEPALIVE int stats_size() { return kStats; }

EMSCRIPTEN_KEEPALIVE int profile_enabled() { return g_profile ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE void set_profile(int on) {
    g_profile = on != 0;
    g_world.profiling = g_profile;
    for (int i = 0; i < 7; ++i) g_stageEma[i] = 0;
}

// Live solver configuration. Reads report whatever the current scene factory
// configured, so the dashboard mirrors the scene rather than inventing defaults.
EMSCRIPTEN_KEEPALIVE int iterations() { return g_world.solver.config().iterations; }
EMSCRIPTEN_KEEPALIVE void set_iterations(int n) {
    if (n < 1) n = 1;
    if (n > 400) n = 400;
    g_world.solver.config().iterations = n;
}

EMSCRIPTEN_KEEPALIVE int position_correction() { return g_world.solver.config().positionCorrection ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE void set_position_correction(int on) {
    g_world.solver.config().positionCorrection = on != 0;
}

EMSCRIPTEN_KEEPALIVE int sleeping_enabled() { return g_world.sleeping ? 1 : 0; }
EMSCRIPTEN_KEEPALIVE void set_sleeping(int on) {
    const bool want = on != 0;
    // Disabling sleep must also release bodies already asleep, or they would sit
    // frozen forever with no integrator touching them.
    if (g_world.sleeping && !want)
        for (auto& b : g_world.bodies) if (!b.isStatic()) b.wake();
    g_world.sleeping = want;
}

EMSCRIPTEN_KEEPALIVE double gravity_y() { return g_world.gravity.y; }
EMSCRIPTEN_KEEPALIVE void set_gravity(double g) {
    if (!std::isfinite(g)) return;
    if (g < -50) g = -50;
    if (g > 50) g = 50;
    g_world.gravity = {0, g};
}

EMSCRIPTEN_KEEPALIVE int solver_mode() {
    return g_world.solver.config().mode == SolverMode::TGS ? 1 : 0;
}
EMSCRIPTEN_KEEPALIVE int sim_steps() { return static_cast<int>(g_steps); }

// Drag a body with a mouse joint, mirroring the native demo's grab.
EMSCRIPTEN_KEEPALIVE int begin_drag(double x, double y) {
    for (std::size_t i = g_world.bodies.size(); i-- > 0; ) {
        if (g_world.bodies[i].isStatic()) continue;
        RigidBody cursor = RigidBody::makeCircle(0.01, 0, {x, y});
        ContactManifold m;
        if (collide(g_world.bodies[i], cursor, m)) {
            const Vec2 local = g_world.bodies[i].rot.transposed() * (Vec2{x, y} - g_world.bodies[i].position);
            g_world.joints.push_back(Joint::mouse(i, local, {x, y}));
            return 1;
        }
    }
    return 0;
}
EMSCRIPTEN_KEEPALIVE void update_drag(double x, double y) {
    if (g_world.joints.empty() || g_world.joints.back().type != JointType::Mouse) return;
    g_world.joints.back().target = {x, y};
}
EMSCRIPTEN_KEEPALIVE void end_drag() {
    if (!g_world.joints.empty() && g_world.joints.back().type == JointType::Mouse)
        g_world.joints.pop_back();
}

} // extern "C"

int main() {
    // No emscripten_set_main_loop here on purpose: the loop is driven by the
    // shell so it keeps working in browsers that starve requestAnimationFrame.
    return 0;
}
