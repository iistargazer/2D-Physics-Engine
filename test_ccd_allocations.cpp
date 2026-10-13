#include "Continuous.hpp"
#include <cstdio>
extern std::size_t testAllocations;
extern bool countTestAllocations;

// Original owning-copy algorithm kept only as a behavioral oracle in this test.
static phys::TOI copyCast(const phys::RigidBody& originalA,
                          const phys::RigidBody& originalB, phys::real duration) {
    using namespace phys;
    TOI result;
    const Separation start = separatingGap(originalA, originalB);
    if (start.gap <= kContactTolerance) {
        result.initiallyTouching = true;
        result.separation = start;
        return result;
    }
    const real speed = closingSpeedBound(originalA, originalB);
    if (speed < kEps) return result;
    RigidBody A = originalA, B = originalB;
    real t = 0;
    for (int iteration = 0; iteration < 128; ++iteration) {
        result.iterations = iteration + 1;
        A.position = originalA.position + originalA.velocity * t;
        B.position = originalB.position + originalB.velocity * t;
        A.setAngle(originalA.angle + originalA.angularVelocity * t);
        B.setAngle(originalB.angle + originalB.angularVelocity * t);
        result.separation = separatingGap(A, B);
        if (result.separation.gap <= kContactTolerance) {
            result.hit = true;
            result.time = t;
            return result;
        }
        const real advance = result.separation.gap / speed;
        if (t + advance > duration) return result;
        t += advance;
    }
    result.converged = false;
    return result;
}

int main() {
    using namespace phys;
    int hits = 0, misses = 0, touching = 0, exhausted = 0;
    for (int i = 0; i < 200; ++i) {
        auto A = i % 3 == 0 ? RigidBody::makeCircle(0.3, 1, {-3, 0})
                            : RigidBody::makeBox(0.6, 0.04, 1, {-3, 0}, 0.12);
        auto B = i % 3 == 1 ? RigidBody::makeCircle(0.5, 1, {})
                            : RigidBody::makeBox(0.02, 8, 0, {});
        A.velocity = {200 + i * 50.0, (i % 7) * 0.2};
        A.angularVelocity = i % 4 == 0 ? 60 : 0;
        B.velocity = i % 5 == 0 ? Vec2{-10, 0} : Vec2{};
        if (i % 10 == 0) A.position = B.position;
        if (i % 10 == 1) A.velocity = {-200, 0};
        testAllocations = 0;
        countTestAllocations = true;
        const auto expected = copyCast(A, B, 1.0 / 60);
        countTestAllocations = false;
        if (!expected.initiallyTouching
            && (A.shape.type == ShapeType::Polygon || B.shape.type == ShapeType::Polygon)
            && testAllocations == 0) {
            std::fprintf(stderr, "allocation counter missed owning copies in cast %d\n", i);
            return 1;
        }
        testAllocations = 0;
        countTestAllocations = true;
        const auto actual = timeOfImpact(A, B, 1.0 / 60);
        countTestAllocations = false;
        if (testAllocations != 0 || actual.hit != expected.hit
            || actual.converged != expected.converged
            || actual.initiallyTouching != expected.initiallyTouching
            || actual.time != expected.time || actual.iterations != expected.iterations
            || actual.separation.gap != expected.separation.gap
            || actual.separation.normal.x != expected.separation.normal.x
            || actual.separation.normal.y != expected.separation.normal.y) {
            std::fprintf(stderr, "cast %d mismatch or allocation: %zu allocations\n", i, testAllocations);
            return 1;
        }
        hits += actual.hit;
        touching += actual.initiallyTouching;
        exhausted += !actual.converged;
        misses += !actual.hit && !actual.initiallyTouching && actual.converged;
    }
    if (hits == 0 || misses == 0 || touching == 0) return 1;
    std::printf("PASS: 200 allocation-free casts match copy oracle exactly; "
                "hit=%d miss=%d touching=%d exhausted=%d; SweepPose=%zu align=%zu\n",
                hits, misses, touching, exhausted, sizeof(SweepPose), alignof(SweepPose));
    return 0;
}
