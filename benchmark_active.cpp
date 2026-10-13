// ============================================================================
// Active-interaction benchmark: what a step costs while the load is working.
//
// benchmark_sleeping measures a settled pile that has gone to sleep. This file
// measures the other regime, which no receipt covered before:
//
//   * 5,000 bodies AWAKE and in persistent contact (sleeping disabled);
//   * the same scene after it sleeps (the separately-accounted frozen-pair cost);
//   * the same scene stirred, so contacts churn instead of merely persisting;
//   * the pre-existing showcase scenes held in motion by the same stir;
//   * a CCD-active case, so the cast stage contributes a real number instead of
//     a rounding error.
//
// Every case reports a per-stage split (broadphase, narrowphase, islands, solver,
// integration, CCD, bookkeeping residual), median and tail frame costs, and the
// wall time the stages do not account for. Three trials per case show spread.
//
// The stir is NOT a scene's natural motion. It is a deterministic (RNG-free)
// per-body force applied every frame so contacts keep churning; it is
// deliberately harsh and is not a claim about any scene's real behaviour.
// ============================================================================
#include "Showcase.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
using namespace phys;

namespace {

constexpr int kTrials = 3;
constexpr real kDt = 1.0 / 60;
constexpr int kStages = 7;
const char* kStageNames[kStages] = {
    "broad", "narrow", "islands", "solve", "integrate", "ccd", "book"};

struct Frame {
    real wall = 0, broad = 0, narrow = 0, islands = 0;
    real solve = 0, integrate = 0, ccd = 0, book = 0;
};

struct Result {
    std::vector<Frame> frames;
    std::size_t bodies = 0, awakeEnd = 0, asleepEnd = 0;
    bool finite = true;
    // Reported instead of thrown: a benchmark that calls step() aborts the whole
    // run on the first capacity mistake, hiding every later case.
    bool capacityExhausted = false;
};

struct Case {
    std::string name;
    World world;
    World::Capacity capacity;
    int warmup = 0, frames = 0;
    bool doStir = false;
    // A sleeping case must actually sleep, or "accounted for separately" is untested.
    std::size_t expectAsleep = 0;
};

// Fixed FMA work. Timed before and after the run so drift in a shared host's
// load is visible in the receipt instead of hiding inside the numbers.
real calibrate() {
    double x = 1.0;
    constexpr int kIterations = 20000000;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) x = x * 1.0000001 + 1e-9;
    const real ms = std::chrono::duration<real, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return std::isfinite(x) ? ms : -1;   // consume x so the loop survives -O2
}

// Deterministic per-body stir: no RNG, so a trial reproduces its own forces.
// Mass-scaled, so every body sees a comparable acceleration.
void stir(World& w, int frame) {
    for (std::size_t i = 0; i < w.bodies.size(); ++i) {
        RigidBody& b = w.bodies[i];
        if (b.isStatic()) continue;
        const real phase = 0.11 * frame + 0.37 * static_cast<real>(i);
        b.applyForce({b.mass * 6.0 * std::sin(phase),
                      b.mass * 2.0 * std::cos(phase * 0.7)}, b.position);
    }
}

// Slow bullets in flight so `advanceContinuous` casts on every frame; the window
// is far too short for them to reach the wall.
World makeCcdStream(int bullets, real speed) {
    World w;
    w.gravity = {};
    auto wall = RigidBody::makeBox(0.05, 400, 0, {0, 0});
    wall.restitution = 0;
    w.add(wall);
    for (int i = 0; i < bullets; ++i) {
        auto b = RigidBody::makeBox(0.10, 0.03, 1, {-300.0 + 2.0 * i, 0.05 * i});
        b.bullet = true; b.restitution = 0; b.velocity = {speed, 0};
        w.add(b);
    }
    return w;
}

Result measure(World w, int warmup, int frames, bool doStir) {
    w.profiling = true;
    for (int f = 0; f < warmup; ++f) {
        if (doStir) stir(w, f);
        if (w.tryStep(kDt) != World::StepStatus::Complete) {
            Result r; r.bodies = w.bodies.size(); r.capacityExhausted = true; return r;
        }
    }
    Result r;
    r.bodies = w.bodies.size();
    r.frames.reserve(static_cast<std::size_t>(frames));
    for (int f = 0; f < frames; ++f) {
        if (doStir) stir(w, f);
        const auto start = std::chrono::steady_clock::now();
        const auto status = w.tryStep(kDt);
        const real wall = std::chrono::duration<real, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        if (status != World::StepStatus::Complete) { r.capacityExhausted = true; break; }
        Frame s;
        s.wall = wall;
        s.broad = w.timings.broadphase; s.narrow = w.timings.narrowphase;
        s.islands = w.timings.islands; s.solve = w.timings.solve;
        s.integrate = w.timings.integrate; s.ccd = w.timings.ccd;
        s.book = w.timings.bookkeeping;
        r.frames.push_back(s);
        for (const auto& b : w.bodies)
            if (!std::isfinite(b.position.lengthSq()) || !std::isfinite(b.velocity.lengthSq()))
                r.finite = false;
    }
    for (const auto& b : w.bodies) {
        if (b.isStatic()) continue;
        if (b.awake) ++r.awakeEnd; else ++r.asleepEnd;
    }
    return r;
}

// Nearest-rank percentile on a copy; the sample count is small and stated.
real pct(std::vector<real> values, real q) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    const std::size_t index = std::min(values.size() - 1,
        static_cast<std::size_t>(q * (values.size() - 1) + 0.5));
    return values[index];
}

real mean(const std::vector<real>& values) {
    if (values.empty()) return 0;
    real total = 0;
    for (const real v : values) total += v;
    return total / values.size();
}

real stageOf(const Frame& f, int stage) {
    switch (stage) {
        case 0: return f.broad;  case 1: return f.narrow;  case 2: return f.islands;
        case 3: return f.solve;  case 4: return f.integrate;
        case 5: return f.ccd;    default: return f.book;
    }
}

// `book` residual attribution: makeStress enables NGS projection, which re-runs
// narrowphase per iteration. This is the same field with projection disabled, kept
// adjacent in the case list so host drift cannot explain the difference.
World makeStressNoProjection(int count) {
    World w = makeStress(count, false);
    w.solver.config().positionCorrection = false;
    return w;
}

std::vector<Case> makeCases() {
    std::vector<Case> cases;
    const World::Capacity stress{5001, 0, 20000, 20000, 20006};
    // The 20-row wrecking pyramid alone is 212 bodies; the bridge adds 100 joints.
    const World::Capacity sceneCapacity{1024, 512, 16384, 16384, 8192};
    cases.push_back({"stress5k_awake_settled", makeStress(5000, false), stress, 180, 120, false, 0});
    cases.push_back({"stress5k_awake_no_ngs", makeStressNoProjection(5000), stress, 180, 120, false, 0});
    cases.push_back({"stress5k_sleeping", makeStress(5000, true), stress, 600, 120, false, 5000});
    cases.push_back({"stress5k_stirred_awake", makeStress(5000, false), stress, 120, 120, true, 0});
    for (const int scene : {0, 1, 2, 3, 4, 5, 6, 9}) {
        std::string name = "showcase0" + std::to_string(scene) + "_stirred";
        cases.push_back({name, makeShowcase(scene), sceneCapacity, 60, 120, true, 0});
    }
    // Scene 7 is the stress field (covered above) and scene 8 is a single CCD
    // event, so it is measured separately here rather than stirred.
    cases.push_back({"ccd_stream_8_bullets", makeCcdStream(8, 40), sceneCapacity, 30, 120, false, 0});
    return cases;
}

}  // namespace

int main() {
    const real calibStart = calibrate();
    std::vector<Case> cases = makeCases();

    std::printf("# active-interaction benchmark; dt=1/60; trials=%d; nearest-rank percentiles; "
                "stir is deterministic and not a scene's natural motion\n", kTrials);
    std::printf("case,trial,bodies,awake_end,asleep_end,frames,mean_ms,p50_ms,p95_ms,p99_ms,max_ms,"
                "unaccounted_ms");
    for (int s = 0; s < kStages; ++s) std::printf(",%s_mean,%s_p99", kStageNames[s], kStageNames[s]);
    std::printf("\n");

    std::vector<std::vector<std::array<real, kStages>>> stageMeans(cases.size());
    std::vector<std::vector<real>> p50s(cases.size());
    bool ok = true;

    for (std::size_t c = 0; c < cases.size(); ++c) {
        Case& item = cases[c];
        World base = std::move(item.world);
        base.prepare(item.capacity);
        for (int trial = 0; trial < kTrials; ++trial) {
            const Result r = measure(base, item.warmup, item.frames, item.doStir);
            std::vector<real> wall;
            for (const auto& f : r.frames) wall.push_back(f.wall);
            std::array<real, kStages> means{};
            std::array<real, kStages> tails{};
            real stageSum = 0;
            for (int s = 0; s < kStages; ++s) {
                std::vector<real> values;
                values.reserve(r.frames.size());
                for (const auto& f : r.frames) values.push_back(stageOf(f, s));
                means[s] = mean(values);
                tails[s] = pct(values, 0.99);
                stageSum += means[s];
            }
            const real wallMean = mean(wall);
            const real p50 = pct(wall, 0.50);
            std::printf("%s,%d,%zu,%zu,%zu,%zu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                        item.name.c_str(), trial, r.bodies, r.awakeEnd, r.asleepEnd,
                        r.frames.size(), wallMean, p50, pct(wall, 0.95), pct(wall, 0.99),
                        pct(wall, 1.0), wallMean - stageSum);
            for (int s = 0; s < kStages; ++s)
                std::printf(",%.6f,%.6f", means[s], tails[s]);
            std::printf("\n");
            std::fflush(stdout);

            if (r.capacityExhausted) {
                std::fprintf(stderr, "FAIL: capacity exhausted in %s; raise that case's capacity\n",
                             item.name.c_str());
                ok = false;
            }
            if (!r.finite) {
                std::fprintf(stderr, "FAIL: non-finite state in %s\n", item.name.c_str());
                ok = false;
            }
            if (item.expectAsleep && r.asleepEnd != item.expectAsleep) {
                std::fprintf(stderr, "FAIL: %s expected %zu asleep, saw %zu\n",
                             item.name.c_str(), item.expectAsleep, r.asleepEnd);
                ok = false;
            }
            stageMeans[c].push_back(means);
            p50s[c].push_back(p50);
        }
    }

    const real calibEnd = calibrate();

    std::fprintf(stderr, "\n%-28s %10s %20s   %-10s %8s\n",
                 "case", "p50 median", "p50 [min..max]", "dominant", "ms");
    for (std::size_t c = 0; c < cases.size(); ++c) {
        std::vector<real> sorted = p50s[c];
        std::sort(sorted.begin(), sorted.end());
        std::array<real, kStages> avg{};
        for (const auto& trial : stageMeans[c])
            for (int s = 0; s < kStages; ++s)
                avg[s] += trial[s] / static_cast<real>(stageMeans[c].size());
        const int top = int(std::max_element(avg.begin(), avg.end()) - avg.begin());
        std::fprintf(stderr, "%-28s %10.3f [%9.3f..%9.3f]   %-10s %8.3f\n",
                     cases[c].name.c_str(), sorted[sorted.size() / 2],
                     sorted.front(), sorted.back(), kStageNames[top], avg[top]);
    }
    std::fprintf(stderr,
        "host-load variance: the fixed 20M-FMA calibration moved %+.1f%% (%.1f -> %.1f ms) across "
        "the run; every figure is single-host wall clock measured while other work shared the machine.\n",
        100 * (calibEnd - calibStart) / calibStart, calibStart, calibEnd);
    return ok ? 0 : 1;
}
