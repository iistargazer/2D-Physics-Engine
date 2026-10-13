#include "World.hpp"
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string_view>

using namespace phys;
using Clock = std::chrono::steady_clock;

static std::vector<RigidBody> scene(std::size_t count, bool dense) {
    std::vector<RigidBody> bodies;
    bodies.reserve(count);
    const std::size_t columns = static_cast<std::size_t>(std::ceil(std::sqrt(real(count))));
    const real spacing = dense ? 0.9 : 2.5;
    for (std::size_t i = 0; i < count; ++i) {
        auto b = RigidBody::makeBox(1, 1, 1,
            {spacing * real(i % columns), spacing * real(i / columns)}, 0.03);
        b.velocity = {0.3, 0.1};
        b.angularVelocity = 0.01;
        bodies.push_back(std::move(b));
    }
    return bodies;
}

struct Measurement { real ms; std::size_t count; };

static Measurement broadphaseTime(std::size_t n, bool dense, bool treeMode) {
    auto bodies = scene(n, dense);
    Broadphase tree;
    tree.findPairs(bodies);
    std::vector<real> times;
    std::size_t hits = 0;
    for (int frame = 0; frame < 15; ++frame) {
        for (auto& b : bodies) { b.position += b.velocity / 60; b.setAngle(b.angle + b.angularVelocity / 60); }
        const auto start = Clock::now();
        hits = 0;
        if (treeMode) hits = tree.findPairs(bodies).size();
        else {
            // Same tight-AABB candidate set, streaming rather than storing the
            // quadratic all-pairs list; include bounds computation in timing.
            std::vector<AABB> bounds;
            bounds.reserve(n);
            for (const auto& b : bodies) bounds.push_back(computeAABB(b));
            for (std::size_t i = 0; i < n; ++i)
                for (std::size_t j = i + 1; j < n; ++j)
                    if (bounds[i].overlaps(bounds[j])) ++hits;
        }
        const real ms = std::chrono::duration<real, std::milli>(Clock::now() - start).count();
        if (frame >= 3) times.push_back(ms);
    }
    if (!tree.tree().validate()) throw std::runtime_error("invalid tree");
    std::sort(times.begin(), times.end());
    return {times[times.size() / 2], hits};
}

static Measurement stepTime(std::size_t n, bool dense, BroadphaseMode mode) {
    World world;
    world.gravity = {};
    world.bodies = scene(n, dense);
    world.broadphaseMode = mode;
    std::vector<real> times;
    for (int frame = 0; frame < 9; ++frame) {
        const auto start = Clock::now();
        world.step(1.0 / 60);
        const real ms = std::chrono::duration<real, std::milli>(Clock::now() - start).count();
        if (frame >= 2) times.push_back(ms);
    }
    std::sort(times.begin(), times.end());
    return {times[times.size() / 2], world.manifolds().size()};
}

int main(int argc, char** argv) {
    const bool quick = argc == 2 && std::string_view(argv[1]) == "--quick";
    std::printf("Compiler: %s; scalar: double; dt: 1/60; median warm-frame timings\n", __VERSION__);
    std::puts("scene,N,tree_broad_ms,brute_AABB_ms,broad_speedup,candidates,tree_step_ms,old_step_ms,step_speedup,contacts");
    for (bool dense : {false, true}) {
        for (std::size_t n : {100u, 1000u, 10000u}) {
            if (quick && n == 10000) continue;
            const auto tree = broadphaseTime(n, dense, true);
            const auto brute = broadphaseTime(n, dense, false);
            const auto step = stepTime(n, dense, BroadphaseMode::DynamicTree);
            const auto old = stepTime(n, dense, BroadphaseMode::AllPairs);
            if (tree.count != brute.count || step.count != old.count) {
                std::fputs("ERROR: oracle mismatch\n", stderr); return 1;
            }
            std::printf("%s,%zu,%.6f,%.6f,%.2f,%zu,%.6f,%.6f,%.2f,%zu\n",
                dense ? "dense" : "sparse", n, tree.ms, brute.ms, brute.ms / tree.ms,
                tree.count, step.ms, old.ms, old.ms / step.ms, step.count);
        }
    }
}
