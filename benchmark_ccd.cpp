#include "World.hpp"
#include <chrono>
#include <cstdio>
using namespace phys;
int main(){
    std::puts("shape,speed,mean_step_ms,max_events,capped_trials");
    for(bool polygon:{false,true})for(real speed:{real(200),real(1000),real(10000)}){
        real total=0;int events=0,caps=0;
        for(int trial=0;trial<100;++trial){
            World w;w.gravity={};auto wall=RigidBody::makeBox(0.02,8,0,{});wall.restitution=0;w.add(wall);
            auto b=polygon?RigidBody::makeBox(0.08,0.02,1,{-2,0},0.1):RigidBody::makeCircle(0.04,1,{-2,0});
            b.bullet=true;b.restitution=0;b.velocity={speed,0};w.add(b);
            const auto start=std::chrono::steady_clock::now();w.step(1.0/60);
            total+=std::chrono::duration<real,std::milli>(std::chrono::steady_clock::now()-start).count();
            events=std::max(events,w.ccdStats.events);caps+=w.ccdStats.capped;
            if(w.bodies[1].position.x>0.1)return 1;
        }
        std::printf("%s,%.0f,%.6f,%d,%d\n",polygon?"polygon":"circle",speed,total/100,events,caps);
    }
}
