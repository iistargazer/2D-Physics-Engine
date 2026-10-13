#include "Showcase.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
using namespace phys;

static real measure(World& w, int frames) {
    const auto start = std::chrono::steady_clock::now();
    for (int f = 0; f < frames; ++f) w.step(1.0 / 60);
    return std::chrono::duration<real, std::milli>(std::chrono::steady_clock::now() - start).count() / frames;
}

int main() {
    std::puts("trial,early_ms,settled_ms,sleep_on_ms,sleep_off_ms,asleep");
    std::array<real, 5> early{}, settled{}, on{}, off{};
    for (std::size_t trial = 0; trial < early.size(); ++trial) {
        auto w = makeStress(5000, true);
        w.prepare({5001, 0, 20000, 20000, 20004});
        early[trial] = measure(w, 30);
        measure(w, 570);
        // Both comparisons start with the same settled poses and cached impulses.
        auto control = w;
        control.prepare({5001, 0, 20000, 20000, 20004});
        control.sleeping = false;
        for (auto& b : control.bodies) if (!b.isStatic()) b.wake();
        settled[trial] = measure(w, 120);
        std::size_t asleep = 0;
        for (const auto& b : w.bodies) {
            if (!std::isfinite(b.position.lengthSq()) || !std::isfinite(b.velocity.lengthSq())) return 2;
            asleep += !b.isStatic() && !b.awake;
        }
        if (asleep != 5000) return 2;
        // Alternate order to reduce consistent temperature/scheduling bias.
        if (trial % 2 == 0) { on[trial] = measure(w, 240); off[trial] = measure(control, 240); }
        else { off[trial] = measure(control, 240); on[trial] = measure(w, 240); }
        for (const auto& b : control.bodies)
            if (!std::isfinite(b.position.lengthSq()) || !std::isfinite(b.velocity.lengthSq())) return 2;
        std::printf("%zu,%.6f,%.6f,%.6f,%.6f,%zu\n", trial,
                    early[trial], settled[trial], on[trial], off[trial], asleep);
        std::fflush(stdout);
    }
    for (auto* sample : {&early, &settled, &on, &off}) std::sort(sample->begin(), sample->end());
    const bool legacy = settled[2] < early[2];
    const bool controlled = on[2] < off[2];
    std::fprintf(stderr, "median legacy settled<early: %s (%.6f<%.6f); "
                         "controlled sleep_on<sleep_off: %s (%.6f<%.6f)\n",
                         legacy ? "PASS" : "FAIL", settled[2], early[2],
                         controlled ? "PASS" : "FAIL", on[2], off[2]);
    // Keep the inherited criterion, even if the controlled comparison passes.
    return legacy && controlled ? 0 : 1;
}
