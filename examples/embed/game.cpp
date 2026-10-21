// ============================================================================
// game.cpp - the smallest honest game loop this engine supports.
//
// This file stands in for the game code of a real title: it owns the clock, the
// input and the spawning, and the engine owns only the simulation. It is
// deliberately outside the engine's headers and needs no build system:
//
//   g++ -std=c++20 -O2 -I. examples/embed/game.cpp -o build/embed_game
//   ./build/embed_game
//
// Expect, byte for byte on a single thread:
//   steps=240 crates=12 contacts=13 y=[-0.500, 2.477] player_x=1.842
// The two crate columns are knocked into a low heap by the player box that walks
// through them, the ground slab sits at y = -0.5 and nothing is left above 2.48 m.
// Nothing here links SFML, allocates a renderer or reads a file; a renderer would
// read `world.bodies` after each step and draw it.
// ============================================================================
#include "World.hpp"

#include <algorithm>
#include <cstdio>

int main() {
    phys::World world;                       // y-up, gravity -9.81 m/s^2 by default
    world.solver.config().iterations = 12;   // game budget: fewer sweeps than the demos

    // Static ground: a slab whose half-extents are 20 x 0.5 m.
    world.add(phys::RigidBody::makeBox(20, 0.5, 0, {0, -0.5}));  // density 0 => static

    // Twelve crates stacked in two columns, spawned by game content, not by the engine.
    for (int i = 0; i < 6; ++i)
        for (int column = 0; column < 2; ++column)
            world.add(phys::RigidBody::makeBox(0.5, 0.5, 1, {column * 1.1 - 0.55, 0.5 + i}));

    // A player body: the game decides its motion, so it is a bullet-shaped box
    // driven by writing velocity, exactly what a platformer controller does.
    const std::size_t player = world.add(phys::RigidBody::makeBox(0.4, 0.8, 1, {0, 7.5}));

    // Fixed-timestep accumulator. The engine is never given a variable dt: the
    // game hands it the same 1/60 s slice however the frame time wandered.
    constexpr double dt = 1.0 / 60.0;
    double accumulator = 0;
    int steps = 0;
    for (double frame : {0.004, 0.021, 0.016, 0.033, 0.009}) {   // a jittery frame clock
        accumulator += frame;
        while (accumulator >= dt) {
            world.bodies[player].velocity.x = 1.5;   // "hold right"
            world.step(dt);
            accumulator -= dt;
            ++steps;
        }
    }
    // Run the rest of the four seconds at a steady 60 Hz.
    for (; steps < 240; ++steps) world.step(dt);

    double lowest = 0, highest = 0;
    for (const auto& body : world.bodies) {
        lowest = std::min(lowest, body.position.y);
        highest = std::max(highest, body.position.y);
    }
    std::printf("steps=%d crates=%zu contacts=%zu y=[%.3f, %.3f] player_x=%.3f\n",
                steps, world.bodies.size() - 2, world.manifolds().size(),
                lowest, highest, world.bodies[player].position.x);
    return 0;
}
