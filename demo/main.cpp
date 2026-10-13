// ============================================================================
// phys2d — SFML showcase.
//
// The renderer is the only third-party dependency in the project: the physics is
// the same header-only core the tests and the WebAssembly build use. Everything
// below is presentation: scene switching, fixed-step accumulation with
// interpolated drawing, debug overlays and PNG capture for slides.
//
// Controls
//   1-9, 0      load scene 1-9, 0 = scene 10 (momentum row)
//   Space       pause / resume
//   N           advance exactly one fixed 1/60 s step (works while paused)
//   S           slow motion (0.2x time)
//   D           debug overlay: AABBs, contact points and normals, joint anchors
//   G           broadphase tree overlay (internal node bounds)
//   H           help line in the window title
//   F1          save a PNG of the current frame under build/
//   R           restart the current scene
//   LMB         drag a body with a mouse spring
//   RMB         spawn a box at the cursor        MMB  spawn a circle
// ============================================================================
#include "Showcase.hpp"
#include "Broadphase.hpp"
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

using namespace phys;

namespace {
constexpr real kScale = 25;                 // pixels per metre
constexpr real kOriginX = 640, kOriginY = 730;
constexpr real kFixedStep = 1.0 / 60;

// One line of narration per scene, shown in the title bar so a viewer can read
// what they are looking at without the presenter talking over it.
constexpr const char* sceneNotes[kSceneCount] = {
    "5 elastic balls on equal-length constraints - the chain passes momentum to the far ball",
    "the nudge propagates down 45 thin dominoes, each one rotating the next",
    "a 20-row pile settles while a 19 m pendulum swings in, then the impact scatters it",
    "revolute joints with angle limits; grab a limb with the left mouse button",
    "100 planks pinned end to end by revolute joints, held by rigid position projection",
    "prismatic axles let the wheels travel vertically and spin freely; the wheels drive",
    "ramp, dominoes, hammer and flag - flipped only when the trigger ball is present",
    "5,000 boxes, one broadphase tree, islands and sleeping in the settled state",
    "a thin bullet at 900 m/s against a hair-thin wall: discrete stepping would tunnel",
    "identical frictionless balls, zero gravity: total momentum is conserved exactly",
};

sf::Color bodyColor(const RigidBody& b) {
    if (b.isStatic()) return sf::Color(80, 100, 132);
    if (!b.awake) return sf::Color(96, 116, 140);
    if (b.bullet) return sf::Color(255, 140, 120);
    return sf::Color(96, 197, 241);
}

void line(sf::RenderWindow& window, sf::Vector2f a, sf::Vector2f b, sf::Color color, float width = 1.0f) {
    const sf::Vector2f d = b - a;
    const float length = std::sqrt(d.x * d.x + d.y * d.y);
    if (length < 0.001f) return;
    sf::RectangleShape shape({length, width});
    shape.setPosition(a);
    shape.setRotation(std::atan2(d.y, d.x) * 180.0f / 3.14159265358979323846f);
    shape.setFillColor(color);
    window.draw(shape);
}

// sf::Texture::update(window) is the portable way to grab the framebuffer; it
// needs no platform-specific headers, which keeps the demo buildable anywhere
// SFML is.
bool saveScreenshot(const sf::RenderWindow& window, const std::string& name) {
    sf::Texture texture;
    const auto size = window.getSize();
    if (!texture.create(size.x, size.y)) return false;
    texture.update(window);
    return texture.copyToImage().saveToFile(name);
}
} // namespace

int main(int argc, char** argv) {
    int scene = argc > 1 ? std::max(0, std::min(kSceneCount - 1, std::atoi(argv[1]) - 1)) : 0;
    sf::RenderWindow window(sf::VideoMode(1280, 800), "phys2d");
    window.setVerticalSyncEnabled(true);

    World world = makeShowcase(scene);
    world.profiling = true;
    bool paused = false, debug = false, treeView = false, slow = false, stepOnce = false, showHelp = false, dragging = false;
    real accumulator = 0, lastMs = 0, smoothedMs = 0;
    sf::Clock frameClock;
    std::vector<Vec2> previousPosition;
    std::vector<real> previousAngle;
    Broadphase tree;

    const auto screen = [](Vec2 p) {
        return sf::Vector2f(float(kOriginX + p.x * kScale), float(kOriginY - p.y * kScale));
    };
    const auto mouseWorld = [&]() {
        const auto p = sf::Mouse::getPosition(window);
        return Vec2{(p.x - kOriginX) / kScale, (kOriginY - p.y) / kScale};
    };

    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed) {
                const auto key = event.key.code;
                if (key == sf::Keyboard::Space) paused = !paused;
                if (key == sf::Keyboard::N) stepOnce = true;
                if (key == sf::Keyboard::D) debug = !debug;
                if (key == sf::Keyboard::G) treeView = !treeView;
                if (key == sf::Keyboard::H) showHelp = !showHelp;
                if (key == sf::Keyboard::S) slow = !slow;
                if (key == sf::Keyboard::R) { world = makeShowcase(scene); world.profiling = true; previousPosition.clear(); previousAngle.clear(); accumulator = 0; dragging = false; }
                if (key == sf::Keyboard::F1) {
                    char name[64];
                    std::snprintf(name, sizeof name, "build/physics_scene_%d.png", scene + 1);
                    std::puts(saveScreenshot(window, name) ? name : "screenshot failed");
                }
                const int digit = key >= sf::Keyboard::Num1 && key <= sf::Keyboard::Num9 ? int(key - sf::Keyboard::Num1)
                                : key == sf::Keyboard::Num0 ? 9 : -1;
                if (digit >= 0) {
                    scene = digit;
                    world = makeShowcase(scene);
                    world.profiling = true;
                    previousPosition.clear(); previousAngle.clear();
                    accumulator = 0; dragging = false;
                }
            }
            if (event.type == sf::Event::MouseButtonPressed) {
                const Vec2 p = mouseWorld();
                if (event.mouseButton.button == sf::Mouse::Right) {
                    auto box = RigidBody::makeBox(0.8, 0.8, 1, p, real(std::rand()) / RAND_MAX * 1.2 - 0.6);
                    box.restitution = 0.15;
                    world.add(box);
                } else if (event.mouseButton.button == sf::Mouse::Middle) {
                    auto ball = RigidBody::makeCircle(0.5, 1, p);
                    ball.restitution = 0.35;
                    world.add(ball);
                } else if (event.mouseButton.button == sf::Mouse::Left) {
                    // Grab the topmost body whose shape actually contains the cursor.
                    for (std::size_t i = world.bodies.size(); i-- > 0;) {
                        if (world.bodies[i].isStatic()) continue;
                        if (world.bodies[i].shape.type == ShapeType::Circle && world.bodies[i].shape.radius < 0.05) continue;
                        RigidBody cursor = RigidBody::makeCircle(0.01, 0, p);
                        ContactManifold m;
                        if (collide(world.bodies[i], cursor, m)) {
                            const Vec2 local = world.bodies[i].rot.transposed() * (p - world.bodies[i].position);
                            world.joints.push_back(Joint::mouse(i, local, p));
                            dragging = true;
                            break;
                        }
                    }
                }
            }
            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left && dragging) {
                if (!world.joints.empty()) world.joints.pop_back();
                dragging = false;
            }
        }
        if (dragging && !world.joints.empty()) world.joints.back().target = mouseWorld();

        const real elapsed = std::min(real(frameClock.restart().asSeconds()), real(0.25));
        if (!paused) accumulator += elapsed * (slow ? 0.2 : 1.0);
        if (stepOnce) { accumulator = kFixedStep; stepOnce = false; }

        while (accumulator >= kFixedStep) {
            previousPosition.clear();
            previousAngle.clear();
            for (const auto& b : world.bodies) { previousPosition.push_back(b.position); previousAngle.push_back(b.angle); }
            const auto start = std::chrono::steady_clock::now();
            world.step(kFixedStep);
            lastMs = real(std::chrono::duration<real, std::milli>(std::chrono::steady_clock::now() - start).count());
            smoothedMs = smoothedMs <= 0 ? lastMs : 0.9 * smoothedMs + 0.1 * lastMs;
            accumulator -= kFixedStep;
        }
        // Interpolate between the last two fixed states so rendering is smooth
        // regardless of how many steps the frame consumed.
        const real alpha = paused ? 1.0 : std::clamp(accumulator / kFixedStep, real(0), real(1));

        window.clear(sf::Color(14, 23, 38));
        for (std::size_t i = 0; i < world.bodies.size(); ++i) {
            const auto& b = world.bodies[i];
            const Vec2 position = i < previousPosition.size() ? previousPosition[i] * (1 - alpha) + b.position * alpha : b.position;
            const real angle = i < previousAngle.size() ? previousAngle[i] * (1 - alpha) + b.angle * alpha : b.angle;
            const Mat22 rotation = Mat22::rotation(angle);
            const sf::Color color = bodyColor(b);
            if (b.shape.type == ShapeType::Circle) {
                sf::CircleShape shape(float(b.shape.radius * kScale));
                shape.setOrigin(shape.getRadius(), shape.getRadius());
                shape.setPosition(screen(position));
                shape.setFillColor(color);
                shape.setOutlineThickness(1.0f);
                shape.setOutlineColor(sf::Color(225, 238, 255, 90));
                window.draw(shape);
                const auto tip = screen(position + rotation * Vec2{b.shape.radius, 0});
                line(window, screen(position), tip, sf::Color(255, 255, 255, 170));
            } else {
                sf::ConvexShape shape(b.shape.verts.size());
                for (std::size_t v = 0; v < b.shape.verts.size(); ++v)
                    shape.setPoint(v, screen(position + rotation * b.shape.verts[v]));
                shape.setFillColor(color);
                shape.setOutlineThickness(1.0f);
                shape.setOutlineColor(sf::Color(225, 238, 255, 90));
                window.draw(shape);
            }
        }

        if (debug || treeView) {
            // Joints: anchor points plus the segment between them. A stretched
            // joint shows up as a visible gap rather than a silent error.
            for (const auto& j : world.joints) {
                if (j.a >= world.bodies.size() || j.b >= world.bodies.size()) continue;
                const auto& A = world.bodies[j.a];
                const auto& B = world.bodies[j.b];
                const Vec2 anchorA = A.position + A.rot * j.localA;
                const Vec2 anchorB = B.position + B.rot * j.localB;
                line(window, screen(anchorA), screen(anchorB), sf::Color(255, 120, 200, 200));
                for (const Vec2& anchor : {anchorA, anchorB}) {
                    sf::CircleShape dot(3.0f);
                    dot.setOrigin(3.0f, 3.0f);
                    dot.setPosition(screen(anchor));
                    dot.setFillColor(sf::Color(255, 120, 200));
                    window.draw(dot);
                }
            }
        }
        if (debug) {
            for (const auto& b : world.bodies) {
                const AABB a = computeAABB(b);
                sf::RectangleShape box(sf::Vector2f(float((a.max.x - a.min.x) * kScale), float((a.max.y - a.min.y) * kScale)));
                box.setPosition(screen({a.min.x, a.max.y}));
                box.setFillColor(sf::Color::Transparent);
                box.setOutlineThickness(1.0f);
                box.setOutlineColor(sf::Color(70, 210, 190, 140));
                window.draw(box);
            }
            // Contact points with normals scaled by accumulated normal impulse:
            // the picture shows where the solver is actually pushing.
            for (const auto& m : world.manifolds()) {
                for (int i = 0; i < m.count; ++i) {
                    const auto& c = m.contactPoints[i];
                    sf::CircleShape dot(3.0f);
                    dot.setOrigin(3.0f, 3.0f);
                    dot.setPosition(screen(c.position));
                    dot.setFillColor(sf::Color(255, 180, 70));
                    window.draw(dot);
                    const Vec2 normal = m.normal * (0.2 + 0.02 * std::min(real(20), c.accN));
                    line(window, screen(c.position), screen(c.position + normal), sf::Color(255, 90, 90), 2.0f);
                }
            }
        }
        if (treeView) {
            // The overlay shows the tree the engine actually built this frame, so
            // it has to be fed the current bounds first.
            tree.findPairs(world.bodies);
            const auto& nodes = tree.tree().nodes();
            for (const auto& n : nodes) {
                if (n.height < 0) continue;
                sf::RectangleShape box(sf::Vector2f(float((n.bounds.max.x - n.bounds.min.x) * kScale),
                                                    float((n.bounds.max.y - n.bounds.min.y) * kScale)));
                box.setPosition(screen({n.bounds.min.x, n.bounds.max.y}));
                box.setFillColor(sf::Color::Transparent);
                box.setOutlineThickness(1.0f);
                box.setOutlineColor(n.height == 0 ? sf::Color(90, 220, 220, 90) : sf::Color(245, 180, 75, 55));
                window.draw(box);
            }
        }

        int awake = 0, bullets = 0;
        for (const auto& b : world.bodies) { if (!b.isStatic() && b.awake) ++awake; if (b.bullet) ++bullets; }
        std::ostringstream title;
        title << "phys2d | " << (scene + 1) << ". " << sceneNames[scene]
              << " | bodies " << world.bodies.size() << " (awake " << awake << ")"
              << " contacts " << world.manifolds().size()
              << " | step " << lastMs << " ms (avg " << smoothedMs << ")"
              << " [broad " << world.timings.broadphase << " narrow " << world.timings.narrowphase
              << " solve " << world.timings.solve << " integrate " << world.timings.integrate << "]"
              << " | " << sceneNotes[scene];
        if (showHelp)
            title << " || 1-9/0 scene / Space pause / N step / S slow / D debug / G tree / R restart / F1 png / LMB drag / RMB box / MMB circle";
        if (paused) title << " || PAUSED";
        if (slow) title << " || SLOW-MO";
        if (bullets) title << " || bullets " << bullets;
        window.setTitle(title.str());
        window.display();
    }
    return 0;
}
