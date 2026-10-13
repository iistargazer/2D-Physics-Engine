#include "Scenes.hpp"
#include <chrono>
#include <cstdio>
using namespace phys;
int main() {
    std::puts("scene,warm,block,iterations,mean_ms,max_late_speed,max_drift,top_y");
    for (bool pyramid : {false, true}) for (bool warm : {false, true}) for (bool block : {false, true}) {
        World w = pyramid ? makePyramid(15, warm) : makeStack(20, warm);
        w.solver.config().blockSolve = block;
        const auto startBodies = w.bodies;
        real speed = 0, drift = 0;
        const auto start = std::chrono::steady_clock::now();
        for (int frame = 0; frame < 1200; ++frame) {
            w.step(1.0 / 60);
            if (frame >= 900) for (std::size_t i = 1; i < w.bodies.size(); ++i) {
                speed = std::max(speed, w.bodies[i].velocity.length());
                drift = std::max(drift, std::abs(w.bodies[i].position.x - startBodies[i].position.x));
                if (!std::isfinite(w.bodies[i].position.lengthSq())) return 1;
            }
        }
        const real ms = std::chrono::duration<real, std::milli>(std::chrono::steady_clock::now()-start).count()/1200;
        std::printf("%s,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n", pyramid ? "pyramid15" : "stack20",
            warm, block, w.solver.config().iterations, ms, speed, drift, w.bodies.back().position.y);
    }
    std::puts("correction,mean_ms,kinetic_energy,residual_depth");
    for(bool ngs:{false,true}) {
        World w;w.gravity={};w.solver.config().positionCorrection=ngs;
        w.add(RigidBody::makeBox(8,1,0,{0,-0.5}));w.add(RigidBody::makeBox(1,1,1,{0,0.3}));
        const auto start=std::chrono::steady_clock::now();
        w.step(1.0/60);
        const auto& b=w.bodies[1];ContactManifold m;
        const real depth=collide(w.bodies[0],w.bodies[1],m)?m.penetration:0;
        const real energy=0.5*b.mass*b.velocity.lengthSq()+0.5*b.inertia*b.angularVelocity*b.angularVelocity;
        const real ms=std::chrono::duration<real,std::milli>(std::chrono::steady_clock::now()-start).count();
        std::printf("%s,%.6f,%.6f,%.6f\n",ngs?"NGS":"Baumgarte",ms,energy,depth);
    }
}
