#include "IntegratorStudy.hpp"
#include <chrono>
#include <cstdio>
#include <fstream>
using namespace phys;
template<class Policy>
void run(const char* name,std::ofstream& out){
    ParticleState s{{1,0},{}};const real dt=0.01,k=100,e0=50;
    real min=e0,max=e0;const auto start=std::chrono::steady_clock::now();
    for(int i=0;i<=100000;++i){
        const real t=i*dt,e=0.5*s.velocity.lengthSq()+0.5*k*s.position.lengthSq();
        min=std::min(min,e);max=std::max(max,e);
        if(i%100==0)out<<name<<','<<t<<','<<e<<','<<(e-e0)/e0<<'\n';
        if(i<100000)Policy::step(s,t,dt,[&](Vec2 x,real){return -k*x;});
    }
    const real ms=std::chrono::duration<real,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::printf("%s: E min=%.9f max=%.9f final=%.9f; 100k steps + CSV %.4f ms\n",name,min,max,
        0.5*s.velocity.lengthSq()+0.5*k*s.position.lengthSq(),ms);
}
int main(){std::ofstream out("build/integrator_energy.csv");if(!out)return 1;
    out<<"policy,time,energy,relative_error\n";
    run<SymplecticPolicy>("Symplectic",out);run<VerletPolicy>("Verlet",out);run<RK4Policy>("RK4",out);
    out.close();return out?0:1;
}
