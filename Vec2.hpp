#pragma once
// ============================================================================
// Vec2 - 2D vector.
//
// Convention: right-handed, y-UP, angles CCW-positive (math convention).
// Flip y only at render time (SFML is y-down).
// ============================================================================
#include <cmath>

namespace phys {

using real = double;                 // switch to float for SIMD/perf later
inline constexpr real kEps = 1e-9;

struct Vec2 {
    real x = 0, y = 0;

    constexpr Vec2() = default;
    constexpr Vec2(real x_, real y_) : x(x_), y(y_) {}

    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(real s) { x *= s; y *= s; return *this; }

    constexpr real lengthSq() const { return x * x + y * y; }
    real length() const { return std::sqrt(lengthSq()); }

    // Returns zero vector for (near-)zero input instead of NaN.
    Vec2 normalized() const {
        const real l = length();
        return l > kEps ? Vec2{x / l, y / l} : Vec2{};
    }

    // 90 degree CCW rotation: (x,y) -> (-y,x).  Equivalent to cross(1, v).
    constexpr Vec2 perp() const { return {-y, x}; }
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, real s) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator*(real s, Vec2 v) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator/(Vec2 v, real s) { return {v.x / s, v.y / s}; }

constexpr real dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

// ---- 2D "cross products" ---------------------------------------------------
// In 3D, a x b is a vector. For vectors living in the xy-plane it points along
// z, so we keep only that scalar: the signed parallelogram area.
//   cross(a,b)  = a.x*b.y - a.y*b.x            (scalar, = |a||b|sin(theta))
// Angular velocity w is a z-vector (0,0,w), so:
//   cross(w, r) = w * perp(r) = (-w*r.y,  w*r.x)   -> velocity of a point at r
//   cross(r, w) = (w*r.y, -w*r.x)                  -> the negation of the above
constexpr real cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
constexpr Vec2 cross(real s, Vec2 v) { return {-s * v.y, s * v.x}; }
constexpr Vec2 cross(Vec2 v, real s) { return {s * v.y, -s * v.x}; }

constexpr real clamp(real v, real lo, real hi) { return v < lo ? lo : (v > hi ? hi : v); }

} // namespace phys
