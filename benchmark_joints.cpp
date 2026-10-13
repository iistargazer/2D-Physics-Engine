#include "Scenes.hpp"
#include <chrono>
#include <cstdio>
using namespace phys;
int main() {
    std::puts("links,iterations,mean_ms,max_anchor_error");
    for (int links : {10, 100}) {
        auto w=makeChain(links); real error=0;
        const auto start=std::chrono::steady_clock::now();
        for(int f=0;f<600;++f) {
            w.step(1.0/60);
            for(const auto& j:w.joints) {
                const auto& a=w.bodies[j.a];const auto& b=w.bodies[j.b];
                error=std::max(error,(b.position+b.rot*j.localB-a.position-a.rot*j.localA).length());
            }
        }
        const real ms=std::chrono::duration<real,std::milli>(std::chrono::steady_clock::now()-start).count()/600;
        std::printf("%d,%d,%.6f,%.6f\n",links,w.solver.config().iterations,ms,error);
        if(!std::isfinite(error))return 1;
    }
}
