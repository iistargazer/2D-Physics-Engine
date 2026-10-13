#pragma once
#include "World.hpp"

namespace phys {
inline World makeStack(int rows = 20, bool warm = true) {
    World w;
    w.solver.config().warmStarting = warm;
    w.solver.config().iterations = 30;
    auto ground = RigidBody::makeBox(60, 1, 0, {0, -0.5});
    ground.restitution = 0; w.add(ground);
    for (int i = 0; i < rows; ++i) {
        auto b = RigidBody::makeBox(1, 1, 1, {0, 0.5 + i});
        b.restitution = 0; w.add(b);
    }
    return w;
}
inline World makePyramid(int rows = 15, bool warm = true) {
    World w;
    w.solver.config().warmStarting = warm;
    w.solver.config().iterations = 30;
    auto ground = RigidBody::makeBox(60, 1, 0, {0, -0.5});
    ground.restitution = 0; w.add(ground);
    for (int row = 0; row < rows; ++row)
        for (int col = 0; col < rows - row; ++col) {
            auto b = RigidBody::makeBox(1, 1, 1,
                {real(col) - real(rows - row - 1) / 2, 0.5 + row});
            b.restitution = 0; w.add(b);
        }
    return w;
}
inline World makeChain(int links = 10) {
    World w; w.solver.config().iterations = 40;
    w.add(RigidBody::makeBox(0.2, 0.2, 0, {0, 10}));
    for (int i=0; i<links; ++i) {
        w.add(RigidBody::makeBox(0.25, 0.8, 1, {0, 9.6-i*0.8}));
        w.joints.push_back(Joint::revolute(std::size_t(i), std::size_t(i+1),
            i==0 ? Vec2{} : Vec2{0,-0.4}, {0,0.4}));
    }
    w.bodies.back().applyImpulse({3,0},{});
    return w;
}
} // namespace phys
