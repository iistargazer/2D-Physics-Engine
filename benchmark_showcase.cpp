// ============================================================================
// Showcase benchmark: every scene factory driven at a fixed 1/60 s.
//
// The scenes are ordinary `World` factories, so this measures exactly what the
// demo and the browser build run. The car scene needs no external torque: the
// wheels carry a persistent `driveTorque`.
// ============================================================================
#include "Showcase.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
using namespace phys;

int main() {
    std::puts("scene,name,bodies,frames,mean_ms,total_ms,max_joint_error,ccd_events,ccd_capped,ccd_clamped,awake_end");
    for (int scene = 0; scene < kSceneCount; ++scene) {
        auto w = makeShowcase(scene);
        // The 5,000-body field needs its full settle-and-sleep window; the other
        // scenes are long past their transient by four seconds.
        const int frames = scene == 7 ? 600 : 240;
        real error = 0;
        int events = 0, capped = 0, clamped = 0;
        const auto start = std::chrono::steady_clock::now();
        for (int f = 0; f < frames; ++f) {
            w.step(1.0 / 60);
            events += w.ccdStats.events;
            capped += w.ccdStats.capped ? 1 : 0;
            clamped += w.ccdStats.clamped;
            for (const auto& b : w.bodies)
                if (!std::isfinite(b.position.lengthSq()) || !std::isfinite(b.velocity.lengthSq())) {
                    std::fprintf(stderr, "nonfinite state in scene %d at frame %d\n", scene, f);
                    return 1;
                }
            for (const auto& j : w.joints)
                if (j.type == JointType::Revolute || j.type == JointType::Weld) {
                    const auto& a = w.bodies[j.a];
                    const auto& b = w.bodies[j.b];
                    error = std::max(error, (b.position + b.rot * j.localB - a.position - a.rot * j.localA).length());
                }
        }
        const real total = std::chrono::duration<real, std::milli>(std::chrono::steady_clock::now() - start).count();
        int awake = 0;
        for (const auto& b : w.bodies) if (!b.isStatic() && b.awake) ++awake;
        std::printf("%d,%s,%zu,%d,%.6f,%.3f,%.6f,%d,%d,%d,%d\n",
            scene, sceneNames[scene], w.bodies.size(), frames, total / frames, total, error,
            events, capped, clamped, awake);
    }
    return 0;
}
