#pragma once
// ============================================================================
// Narrowphase: SAT + clipping manifold generation.
//
// SAT (convex shapes): two convex sets are disjoint  <=>  there exists a
// separating axis (hyperplane separation theorem). For convex POLYGONS in 2D
// the only axes worth testing are the face normals of both polygons (the
// edge-edge cross-product axes of 3D collapse into face normals in 2D).
//
// For an axis n (unit), project both polygons: intervals [minA,maxA], [minB,maxB].
// Overlap along n = min(maxA,maxB) - max(minA,minB).  If ANY axis has overlap<=0
// -> no collision. Otherwise the axis of MINIMUM overlap is the contact normal
// (minimum translation vector direction) and that overlap is the penetration.
//
// Implementation trick (face-based SAT): for each face i of A with outward
// normal n_i and vertex v_i, the signed separation of B along that face is
//        s_i = min_j  n_i . (b_j - v_i)          (B's support point in -n_i)
// s_i > 0 means a gap -> separated. The best axis is argmax_i s_i (least
// negative = least penetration). Do it for A's faces and B's faces; the larger
// is the REFERENCE face, and the other polygon supplies the INCIDENT edge.
//
// Contact points: clip the incident edge against the side planes of the
// reference face, keep the points that lie below the reference face.
// That yields 1 or 2 points (a face-face contact gives 2: this is what makes
// boxes rest stably - the 2 points produce a restoring torque).
//
// Convention: manifold.normal points from A to B.
// ============================================================================
#include "Manifold.hpp"
#include <limits>

namespace phys {

namespace detail {

constexpr real kFaceTolerance = 1e-4;   // prefer A's face on near-ties (coherence)

// Max over A's faces of (min over B's vertices of signed distance to that face).
inline real findMaxSeparation(const RigidBody& A, const RigidBody& B, std::size_t& faceIdx) {
    real best = -std::numeric_limits<real>::infinity();
    const std::size_t na = A.shape.verts.size(), nb = B.shape.verts.size();
    for (std::size_t i = 0; i < na; ++i) {
        const Vec2 n = A.worldNormal(i);
        const Vec2 v = A.worldVertex(i);
        real sep = std::numeric_limits<real>::infinity();
        for (std::size_t j = 0; j < nb; ++j)
            sep = std::min(sep, dot(n, B.worldVertex(j) - v));
        if (sep > best) { best = sep; faceIdx = i; }
    }
    return best;
}

// Sutherland-Hodgman against one half-plane  dot(n,p) - c <= 0.
inline int clip(Vec2 n, real c, Vec2 face[2], std::uint64_t ids[2] = nullptr,
                std::uint64_t crossingId = 0) {
    Vec2 out[2] = {face[0], face[1]};
    std::uint64_t outIds[2] = {ids ? ids[0] : 0, ids ? ids[1] : 0};
    int sp = 0;
    const real d1 = dot(n, face[0]) - c;
    const real d2 = dot(n, face[1]) - c;
    if (d1 <= 0) { outIds[sp] = ids ? ids[0] : 0; out[sp++] = face[0]; }
    if (d2 <= 0) { outIds[sp] = ids ? ids[1] : 0; out[sp++] = face[1]; }
    if (d1 * d2 < 0) {                                   // edge crosses the plane
        const real alpha = d1 / (d1 - d2);
        outIds[sp] = crossingId;
        out[sp++] = face[0] + alpha * (face[1] - face[0]);
    }
    face[0] = out[0];
    face[1] = out[1];
    if (ids) { ids[0] = outIds[0]; ids[1] = outIds[1]; }
    return sp;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Circle vs Circle
// ---------------------------------------------------------------------------
inline bool collideCircles(RigidBody& A, RigidBody& B, ContactManifold& m) {
    const Vec2 d = B.position - A.position;
    const real r = A.shape.radius + B.shape.radius;
    const real distSq = d.lengthSq();
    if (distSq >= r * r) return false;

    const real dist = std::sqrt(distSq);
    m.count = 1;
    if (dist < kEps) {                                   // concentric: pick any normal
        m.normal = {1, 0};
        m.penetration = A.shape.radius;
        m.contactPoints[0] = {A.position, m.penetration};
    } else {
        m.normal = d / dist;
        m.penetration = r - dist;
        m.contactPoints[0] = {A.position + m.normal * A.shape.radius, m.penetration};
    }
    return true;
}

// ---------------------------------------------------------------------------
// Polygon (P) vs Circle (C). Normal points P -> C.
// Voronoi regions: the circle centre is nearest to a face, or to one vertex.
// ---------------------------------------------------------------------------
inline bool collidePolygonCircle(RigidBody& P, RigidBody& C, ContactManifold& m) {
    const real radius = C.shape.radius;
    const Vec2 center = P.rot.transposed() * (C.position - P.position);   // into P's frame

    real sep = -std::numeric_limits<real>::infinity();
    std::size_t face = 0;
    const std::size_t n = P.shape.verts.size();
    for (std::size_t i = 0; i < n; ++i) {
        const real s = dot(P.shape.normals[i], center - P.shape.verts[i]);
        if (s > radius) return false;                    // separating axis found
        if (s > sep) { sep = s; face = i; }
    }

    const Vec2 v1 = P.shape.verts[face];
    const Vec2 v2 = P.shape.verts[(face + 1) % n];
    m.count = 1;
    const std::uint64_t faceId = (std::uint64_t(face) << 2) | 1;

    if (sep < kEps) {                                    // centre inside polygon
        const Vec2 nw = P.rot * P.shape.normals[face];
        m.normal = nw;
        m.penetration = radius - sep;
        m.contactPoints[0] = {C.position - nw * sep, m.penetration};
        m.contactPoints[0].featureId = faceId;
        return true;
    }

    const real d1 = dot(center - v1, v2 - v1);
    const real d2 = dot(center - v2, v1 - v2);
    if (d1 <= 0 || d2 <= 0) {                            // vertex region
        const Vec2 v = d1 <= 0 ? v1 : v2;
        const Vec2 d = center - v;
        const real dist = d.length();
        if (dist > radius) return false;
        m.normal = (P.rot * d).normalized();
        m.penetration = radius - dist;
        m.contactPoints[0] = {P.position + P.rot * v, m.penetration};
        m.contactPoints[0].featureId = (std::uint64_t(d1 <= 0 ? face : (face + 1) % n) << 2) | 2;
    } else {                                             // face region
        const Vec2 nw = P.rot * P.shape.normals[face];
        m.normal = nw;
        m.penetration = radius - sep;
        m.contactPoints[0] = {C.position - nw * sep, m.penetration};
        m.contactPoints[0].featureId = faceId;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Polygon vs Polygon (SAT + clipping). Normal points A -> B.
// ---------------------------------------------------------------------------
inline bool collidePolygons(RigidBody& A, RigidBody& B, ContactManifold& m) {
    std::size_t faceA = 0, faceB = 0;
    const real sepA = detail::findMaxSeparation(A, B, faceA);
    if (sepA >= 0) return false;                         // separating axis on A
    const real sepB = detail::findMaxSeparation(B, A, faceB);
    if (sepB >= 0) return false;                         // separating axis on B

    const bool flip = sepB > sepA + detail::kFaceTolerance;
    const RigidBody& ref = flip ? B : A;
    const RigidBody& inc = flip ? A : B;
    const std::size_t refFace = flip ? faceB : faceA;

    const Vec2 refN = ref.worldNormal(refFace);

    // Incident face: the face of `inc` most anti-parallel to the reference normal.
    const std::size_t ni = inc.shape.verts.size();
    std::size_t incFace = 0;
    real minDot = std::numeric_limits<real>::infinity();
    for (std::size_t i = 0; i < ni; ++i) {
        const real d = dot(refN, inc.worldNormal(i));
        if (d < minDot) { minDot = d; incFace = i; }
    }
    Vec2 face[2] = {inc.worldVertex(incFace), inc.worldVertex((incFace + 1) % ni)};
    // Encode reference owner/face, incident edge, and endpoint or clipping
    // side. A clipped intersection keeps its side identity as it slides.
    const std::uint64_t base = (std::uint64_t(flip) << 63)
        | (std::uint64_t(refFace) << 32) | (std::uint64_t(incFace) << 3);
    std::uint64_t ids[2] = {base, base | 1};

    // Reference face edge and its side planes.
    const std::size_t nr = ref.shape.verts.size();
    const Vec2 v1 = ref.worldVertex(refFace);
    const Vec2 v2 = ref.worldVertex((refFace + 1) % nr);
    const Vec2 side = (v2 - v1).normalized();

    if (detail::clip(-side, -dot(side, v1), face, ids, base | 2) < 2) return false;
    if (detail::clip(side, dot(side, v2), face, ids, base | 3) < 2) return false;

    const real refC = dot(refN, v1);
    m.normal = flip ? -refN : refN;                      // always A -> B
    m.count = 0;
    m.penetration = 0;
    for (int point = 0; point < 2; ++point) {
        const Vec2 p = face[point];
        const real depth = refC - dot(refN, p);          // > 0 => below reference face
        if (depth >= 0) {
            m.contactPoints[m.count] = {p, depth};
            m.contactPoints[m.count++].featureId = ids[point];
            m.penetration = std::max(m.penetration, depth);
        }
    }
    return m.count > 0;
}

// ---------------------------------------------------------------------------
// Dispatch. Returns true and fills m (bodyA/bodyB/normal/points) on contact.
// OBB is just a 4-gon, so it goes through the polygon path.
// ---------------------------------------------------------------------------
inline bool collide(RigidBody& A, RigidBody& B, ContactManifold& m) {
    m.bodyA = &A;
    m.bodyB = &B;
    m.count = 0;
    const bool ca = A.shape.type == ShapeType::Circle;
    const bool cb = B.shape.type == ShapeType::Circle;
    if (ca && cb) return collideCircles(A, B, m);
    if (!ca && !cb) return collidePolygons(A, B, m);
    if (!ca && cb) return collidePolygonCircle(A, B, m);
    if (collidePolygonCircle(B, A, m)) { m.normal = -m.normal; return true; }  // keep A -> B
    return false;
}

} // namespace phys
