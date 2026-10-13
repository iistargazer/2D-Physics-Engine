#include "World.hpp"
#include "Scenes.hpp"
#include "Showcase.hpp"
#include <string_view>
#include <fstream>
#include <iostream>

using namespace phys;

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "build/broadphase_demo.html";
    std::ofstream out(path);
    if (!out) { std::cerr << "Cannot write demo output\n"; return 1; }
    World world;
    world.add(RigidBody::makeBox(36, 1, 0, {0, -0.5}));
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 12; ++col)
            world.add(RigidBody::makeBox(0.8, 0.8, 1,
                {-12 + col * 2.1, 2 + row * 2.0}, 0.12 * (col % 3 - 1)));
    const bool tall = argc > 2 && std::string_view(argv[2]) == "pyramid";
    if (tall) world = makePyramid();
    if (argc > 2 && std::string_view(argv[2]) == "chain") world = makeChain();
    if (argc > 2 && std::string_view(argv[2]) == "ngs") {
        world = makeStack(20); world.solver.config().positionCorrection = true; world.sleeping = true;
    }
    if (argc > 2 && std::string_view(argv[2]) == "bridge") world = makeShowcase(4);
    if (argc > 2 && std::string_view(argv[2]) == "domino") world = makeShowcase(1);
    Broadphase debug;
    out << R"HTML(<!doctype html><html lang="en"><meta charset="utf-8">
<title>phys2d — Dynamic AABB tree</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{margin:0;background:#0e1726;color:#dbe8ff;font:16px system-ui}main{max-width:1100px;margin:24px auto;padding:0 18px}h1{font-size:26px}button,label{margin-right:16px}button{background:#293e5b;color:white;border:0;border-radius:6px;padding:10px 16px}canvas{width:100%;background:#111f32;border:1px solid #304764;border-radius:10px;margin:18px 0}p{color:#a6bcd7}#hud{font-family:monospace}</style>
<main><h1>phys2d / Dynamic AABB tree</h1><p>Actual C++ simulation replay • y-up core • fat leaves in cyan, internal bounds in amber</p>
<button id="pause">Pause</button><button id="restart">Restart</button><label><input id="tree" type="checkbox" checked>Internal nodes</label><label><input id="fat" type="checkbox" checked>Fat AABBs</label><div id="hud"></div><canvas width="1100" height="480"></canvas><p>Fixed 60 Hz C++ scene (falling boxes or 15-row pyramid). This is an exported replay, not a browser/WASM engine or SFML renderer.</p></main><script>
const frames=[
)HTML";
    for (int frame = 0; frame < 240; ++frame) {
        world.step(1.0 / 60);
        const auto& pairs = debug.findPairs(world.bodies);
        if (!debug.tree().validate()) { std::cerr << "Tree invalid\n"; return 1; }
        if (frame) out << ",\n";
        out << "{h:" << debug.tree().height() << ",p:" << pairs.size()
            << ",r:" << debug.reinserts() << ",c:" << world.manifolds().size() << ",b:[";
        for (std::size_t i = 0; i < world.bodies.size(); ++i) {
            if (i) out << ',';
            out << '[';
            const auto& b = world.bodies[i];
            for (std::size_t j = 0; j < b.shape.verts.size(); ++j) {
                if (j) out << ',';
                const auto p = b.worldVertex(j);
                out << '[' << p.x << ',' << p.y << ']';
            }
            out << ']';
        }
        out << "],n:[";
        bool first = true;
        for (const auto& n : debug.tree().nodes()) {
            if (n.height < 0) continue;
            if (!first) out << ',';
            first = false;
            out << '[' << n.bounds.min.x << ',' << n.bounds.min.y << ','
                << n.bounds.max.x << ',' << n.bounds.max.y << ',' << n.height << ']';
        }
        out << "]}";
    }
    out << R"HTML(];
const canvas=document.querySelector('canvas'),ctx=canvas.getContext('2d');
const pause=document.querySelector('#pause'),hud=document.querySelector('#hud');
let running=true,index=0,last=performance.now(),acc=0;
pause.onclick=()=>{running=!running;pause.textContent=running?'Pause':'Play'};
document.querySelector('#restart').onclick=()=>{index=0;acc=0};
const scale=frames[0].b.length>100?25:frames[0].b.length===21?19:28;
const point=([x,y])=>[550+x*scale,420-y*scale];
function draw(){const f=frames[index];ctx.clearRect(0,0,1100,480);
for(const n of f.n){if(n[4]===0?!document.querySelector('#fat').checked:!document.querySelector('#tree').checked)continue;
const p=point([n[0],n[3]]);ctx.strokeStyle=n[4]===0?'#51d8d899':'#f5b24c33';ctx.lineWidth=1;ctx.strokeRect(p[0],p[1],(n[2]-n[0])*scale,(n[3]-n[1])*scale)}
f.b.forEach((verts,i)=>{ctx.beginPath();verts.forEach((v,j)=>{const p=point(v);j?ctx.lineTo(...p):ctx.moveTo(...p)});ctx.closePath();ctx.fillStyle=i===0?'#526984':['#60c5f1','#b391ef','#6dd8bb'][i%3];ctx.fill();ctx.strokeStyle='#e1eeff';ctx.stroke()});
hud.textContent=`Frame ${index+1}/240 | Bodies ${f.b.length} | Tree height ${f.h} | Candidates ${f.p} | Contacts ${f.c} | Reinserts ${f.r}`;
}
function tick(now){if(running){acc+=Math.min(now-last,250);while(acc>=1000/60){index=(index+1)%frames.length;acc-=1000/60}}last=now;draw();requestAnimationFrame(tick)}
draw();requestAnimationFrame(tick);
</script></html>)HTML";
    out.close();
    if (!out) { std::cerr << "Demo write failed\n"; return 1; }
    std::cout << "Exported 240 fixed-step frames to " << path << '\n';
}
