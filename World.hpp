#pragma once
// ============================================================================
// World - owns bodies, runs one fixed timestep.
//
//   1. integrateForces      v += (F/m + g) dt          (symplectic step A)
//   2. broadphase + narrowphase -> manifolds
//   3. solver.prepare / solve   (N sequential-impulse sweeps)
//   4. integrateVelocities  x += v dt                  (symplectic step B)
//
// Broadphase: balanced dynamic AABB tree; AllPairs remains a debug/test oracle.
// NOTE: manifolds hold raw RigidBody*; do not add bodies during step().
// ============================================================================
#include "Collision.hpp"
#include "Broadphase.hpp"
#include "Integrator.hpp"
#include "Solver.hpp"
#include "TemporalSolver.hpp"
#include "SolverRows.hpp"
#include "Joint.hpp"
#include "Islands.hpp"
#include "Continuous.hpp"
#include <limits>
#include <stdexcept>
#include <chrono>

namespace phys {

class World {
public:
    Vec2 gravity{0, -9.81};          // y-up
    std::vector<RigidBody> bodies;
    ContactSolver solver;
    std::vector<Joint> joints;
    bool sleeping = false;
    CCDStats ccdStats;
    // Per-frame stage split. `integrate` is the integrator only (forces then poses
    // or velocities); CCD, islands and a `bookkeeping` residual (row build, NGS
    // projection, impulse-cache save) are separate, so the stages sum toward wall
    // time instead of hiding unrelated work inside one bucket.
    struct Timings {
        real broadphase = 0, narrowphase = 0, islands = 0;
        real solve = 0, integrate = 0, ccd = 0, bookkeeping = 0;
    } timings;
    bool profiling = false;
    const Islands& islands() const { return islands_; }
    BroadphaseMode broadphaseMode = BroadphaseMode::DynamicTree;

    std::size_t add(RigidBody b) { bodies.push_back(std::move(b)); return bodies.size() - 1; }

    struct Capacity {
        std::size_t bodies = 0, joints = 0, pairs = 0, contacts = 0, vertices = 0;
    };
    enum class StepStatus { Complete, Bodies, Joints, Vertices, Pairs, Contacts };

    // Setup only. Counts are logical limits, independent of vector over-reservation.
    void prepare(Capacity capacity) {
        if (capacity.bodies > static_cast<std::size_t>(std::numeric_limits<int>::max() / 2))
            throw std::length_error("tree body capacity exceeds int indices");
        bodies.reserve(capacity.bodies); joints.reserve(capacity.joints);
        broadphase_.reserve(capacity.bodies, capacity.pairs);
        islands_.reserve(capacity.bodies);
        manifolds_.reserve(capacity.contacts); nextManifolds_.reserve(capacity.contacts);
        cache_.reserve(capacity.contacts); snapshots_.reserve(capacity.bodies);
        temporal_.reserve(capacity.contacts);
        rows_.reserve(capacity.joints + capacity.contacts);
        vertices_.reserve(capacity.vertices); ccdScratch_.reserve(capacity.bodies);
        capacity_ = capacity; prepared_ = true;
    }

    void step(real dt) {
        if (tryStep(dt) != StepStatus::Complete)
            throw std::length_error("prepared World capacity exhausted; use tryStep for allocation-free status");
    }

    StepStatus tryStep(real dt) {
        if (!(dt > 0) || !std::isfinite(dt)) throw std::invalid_argument("step requires positive finite dt");
        const auto& cfg = solver.config();
        if (cfg.mode == SolverMode::TGS
            && (cfg.substeps < 1 || cfg.iterations < 1 || !(dt / cfg.substeps > 0)
                || !std::isfinite(cfg.compliance) || cfg.compliance < 0
                || !std::isfinite(cfg.damping) || cfg.damping < 0
                || !std::isfinite(temporalSoftness(cfg, dt / cfg.substeps).gamma)
                || !std::isfinite(temporalSoftness(cfg, dt / cfg.substeps).beta)))
            throw std::invalid_argument("invalid temporal solver configuration");
        for (const auto& joint : joints) {
            if (joint.a >= bodies.size() || joint.b >= bodies.size()) throw std::out_of_range("joint body index");
            if (joint.limits && joint.lower > joint.upper) throw std::invalid_argument("reversed joint limits");
        }
        if (prepared_) {
            if (bodies.size() > capacity_.bodies) return StepStatus::Bodies;
            if (joints.size() > capacity_.joints) return StepStatus::Joints;
            std::size_t count = 0;
            for (const auto& b : bodies) {
                if (b.shape.verts.size() > capacity_.vertices - count) return StepStatus::Vertices;
                count += b.shape.verts.size();
            }
        }
        const auto tick = [] { return std::chrono::steady_clock::now(); };
        const auto elapsed = [&](auto start) { return std::chrono::duration<real,std::milli>(tick()-start).count(); };
        const bool changed = previousMode_ != cfg.mode || cacheChanged();
        nextManifolds_.clear();
        const StepStatus status = findPairs(!changed);
        if (status != StepStatus::Complete) return status;
        if (changed) {
            cache_.clear();
            for (auto& joint : joints) joint.resetImpulses();
            if (sleeping) for (auto& b : bodies) if (!b.isStatic()) b.wake();
        }
        // Island pass first, on the previous frame's contacts and post-solve
        // velocities: that is what makes sleeping stable (see Islands.hpp).
        if (sleeping) {
            if (changed) {
                // Old indices may refer to erased/reordered public bodies.
                manifolds_.clear();
            }
            const auto islandStart = tick();
            islands_.build(bodies, manifolds_, joints);
            islands_.wakeConnected(bodies, joints);
            if (profiling) timings.islands += elapsed(islandStart);
        }
        // Previous manifolds remain available to the wake graph until this swap.
        manifolds_.swap(nextManifolds_);
        // The row list is the addressing seam a parallel schedule is computed
        // from. Built here, once, so the solve and the schedule see one order.
        const auto rowStart = tick();
        rows_.build(manifolds_, joints, bodies);
        if (profiling) timings.bookkeeping += elapsed(rowStart);
        if (cfg.mode == SolverMode::TGS) {
            stepTemporal(dt);
            if (sleeping) {
                const auto sleepStart = tick();
                islands_.sleep(bodies, dt);
                if (profiling) timings.islands += elapsed(sleepStart);
            }
            const auto saveStart = tick();
            saveImpulses(dt / cfg.substeps);
            if (profiling) timings.bookkeeping += elapsed(saveStart);
            previousMode_ = cfg.mode;
            return StepStatus::Complete;
        }
        auto phaseStart = tick();
        for (auto& b : bodies) if (!sleeping || b.awake) integrateForces(b, gravity, dt);

        if (profiling) timings.integrate += elapsed(phaseStart);
        phaseStart = tick();

        restoreImpulses(dt);
        solver.prepare(manifolds_, dt);
        for (auto& joint : joints) if (!sleeping || bodies[joint.a].awake || bodies[joint.b].awake) joint.prepare(bodies, dt,
            solver.config().warmStarting && previousDt_ > 0 ? dt / previousDt_ : 0);
        for (int iteration = 0; iteration < solver.config().iterations; ++iteration) {
            for (auto& joint : joints) if (!sleeping || bodies[joint.a].awake || bodies[joint.b].awake) joint.solve(bodies);
            solver.solveIteration(manifolds_);
        }

        if (profiling) timings.solve += elapsed(phaseStart);
        phaseStart = tick();
        const bool bullets = std::any_of(bodies.begin(),bodies.end(),[](const auto& b){return b.bullet;});
        ccdStats = {};
        if (bullets) {
            ccdStats = advanceContinuous(bodies,dt,ccdScratch_);
            if (profiling) timings.ccd += elapsed(phaseStart);
        } else {
            for (auto& b : bodies) {
                if (!sleeping || b.awake) integrateVelocities(b, dt);
                else { b.force = {}; b.torque = 0; }
            }
            if (profiling) timings.integrate += elapsed(phaseStart);
        }
        // Order is load-bearing: projection, then sleep, then snapshot.
        phaseStart = tick();
        if (solver.config().positionCorrection) correctPositions();
        if (profiling) timings.bookkeeping += elapsed(phaseStart);
        phaseStart = tick();
        if (sleeping) islands_.sleep(bodies,dt);
        if (profiling) timings.islands += elapsed(phaseStart);
        phaseStart = tick();
        saveImpulses(dt);
        previousMode_ = cfg.mode;
        if (profiling) timings.bookkeeping += elapsed(phaseStart);
        return StepStatus::Complete;
    }

    void clearContactCache() { cache_.clear(); snapshots_.clear(); }

    // Index-stable rows for the tick just simulated; see SolverRows.hpp.
    const RowSet& rows() const { return rows_; }
    // Which bodies the solver may write. This is the same predicate the writes
    // are guarded by, so a schedule and the guards cannot disagree.
    std::vector<char> movableBodies() const {
        std::vector<char> movable(bodies.size(), 0);
        for (std::size_t i = 0; i < bodies.size(); ++i)
            movable[i] = !bodies[i].isStatic();
        return movable;
    }

    const std::vector<ContactManifold>& manifolds() const { return manifolds_; }

private:
    void stepTemporal(real dt) {
        const auto& cfg = solver.config();
        const real h = dt / cfg.substeps;
        const bool bullets = std::any_of(bodies.begin(), bodies.end(), [](const auto& b) { return b.bullet; });
        ccdStats = {};
        restoreImpulses(h);
        // Capture restitution after the first force slice, before any warm impulse.
        for (int substep = 0; substep < cfg.substeps; ++substep) {
            const auto forceStart = std::chrono::steady_clock::now();
            for (auto& b : bodies) if (!sleeping || b.awake) integrateForces(b, gravity, h);
            if (profiling) timings.integrate += std::chrono::duration<real, std::milli>(
                std::chrono::steady_clock::now() - forceStart).count();
            const auto start = std::chrono::steady_clock::now();
            if (substep == 0) prepareTemporal(manifolds_, temporal_, cfg);
            refreshTemporal(manifolds_, temporal_, cfg, h, substep == 0);
            const real ratio = cfg.warmStarting
                ? (substep == 0 ? (previousDt_ > 0 ? h / previousDt_ : 0) : 1) : 0;
            for (auto& joint : joints)
                if (!sleeping || bodies[joint.a].awake || bodies[joint.b].awake) joint.prepare(bodies, h, ratio);
            for (int it = 0; it < cfg.iterations; ++it) {
                for (auto& joint : joints)
                    if (!sleeping || bodies[joint.a].awake || bodies[joint.b].awake) joint.solve(bodies);
                solveTemporal(manifolds_, cfg, h);
            }
            if (profiling) timings.solve += std::chrono::duration<real, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            const auto integrateStart = std::chrono::steady_clock::now();
            if (bullets) {
                const auto stats = advanceContinuous(bodies, h, ccdScratch_, 64, false);
                ccdStats.events += stats.events; ccdStats.casts += stats.casts;
                ccdStats.skipped += stats.skipped; ccdStats.unconverged += stats.unconverged;
                ccdStats.pushedOut += stats.pushedOut; ccdStats.clamped += stats.clamped;
                ccdStats.capped = ccdStats.capped || stats.capped;
                if (profiling) timings.ccd += std::chrono::duration<real, std::milli>(
                    std::chrono::steady_clock::now() - integrateStart).count();
            } else {
                for (auto& b : bodies) if (!b.isStatic() && (!sleeping || b.awake)) {
                    b.position += b.velocity * h;
                    b.setAngle(b.angle + b.angularVelocity * h);
                }
                if (profiling) timings.integrate += std::chrono::duration<real, std::milli>(
                    std::chrono::steady_clock::now() - integrateStart).count();
            }
        }
        const auto clearStart = std::chrono::steady_clock::now();
        for (auto& b : bodies) { b.force = {}; b.torque = 0; }
        if (profiling) timings.bookkeeping += std::chrono::duration<real, std::milli>(
            std::chrono::steady_clock::now() - clearStart).count();
    }

    bool connected(std::size_t i, std::size_t j) const {
        for (const auto& joint : joints)
            if (joint.type != JointType::Mouse && !joint.collideConnected
                && ((joint.a == i && joint.b == j) || (joint.a == j && joint.b == i))) return true;
        return false;
    }
    // True when a body cannot have moved since the last frame: asleep, or static
    // (never moves). Used to reuse the previous frame's contact instead of
    // re-running the whole broadphase/narrowphase pipeline for it.
    static bool inactive(const RigidBody& b) { return b.isStatic() || !b.awake; }

    StepStatus findPairs(bool reuseFrozen) {
        timings = {};
        const auto emit = [&](const ContactManifold& m) {
            if (prepared_ && nextManifolds_.size() == capacity_.contacts) return false;
            nextManifolds_.push_back(m);
            return true;
        };
        const auto start = std::chrono::steady_clock::now();
        if (broadphaseMode == BroadphaseMode::AllPairs) {
            // Stream the original oracle: do not allocate N(N-1)/2 pairs.
            for (std::size_t i = 0; i < bodies.size(); ++i)
                for (std::size_t j = i + 1; j < bodies.size(); ++j) {
                    if ((bodies[i].isStatic() && bodies[j].isStatic()) || connected(i, j)) continue;
                    ContactManifold m;
                    if (collide(bodies[i], bodies[j], m)) {
                        m.indexA = i; m.indexB = j;
                        if (!emit(m)) return StepStatus::Contacts;
                    }
                }
            if (profiling) timings.narrowphase = std::chrono::duration<real,std::milli>(std::chrono::steady_clock::now()-start).count();
            return StepStatus::Complete;
        }
        // ---- frozen pairs -------------------------------------------------------
        // A pair where neither body can have moved since the last frame has an
        // unchanged contact. Reuse the cached manifold verbatim instead of paying a
        // tree query and a narrowphase call for it. This is what makes a settled
        // 5,000-body scene cheap: its contact graph is frozen, not recomputed.
        if (sleeping && reuseFrozen) {
            for (const auto& cached : cache_) {
                const BodyPair pair{cached.indexA, cached.indexB};
                if (pair.first >= bodies.size() || pair.second >= bodies.size()) continue;
                RigidBody& A = bodies[pair.first];
                RigidBody& B = bodies[pair.second];
                if (A.isStatic() && B.isStatic()) continue;
                if (!inactive(A) || !inactive(B)) continue;
                if (cached.count <= 0) continue;
                ContactManifold m;
                m.bodyA = &A; m.bodyB = &B;
                m.indexA = pair.first; m.indexB = pair.second;
                m.normal = cached.normal;
                m.penetration = cached.penetration;
                m.friction = cached.friction;
                m.restitution = cached.restitution;
                m.count = cached.count;
                m.contactPoints = cached.contactPoints;
                if (!emit(m)) return StepStatus::Contacts;
            }
        }
        const auto& pairs = broadphase_.findPairs(bodies, broadphaseMode,
            prepared_ ? capacity_.pairs : std::numeric_limits<std::size_t>::max(), !reuseFrozen);
        if (broadphase_.exhausted()) return StepStatus::Pairs;
        const auto narrowStart = std::chrono::steady_clock::now();
        if (profiling) timings.broadphase = std::chrono::duration<real,std::milli>(narrowStart-start).count();
        for (const auto& [i, j] : pairs) {
            if (connected(i, j)) continue;
            if (reuseFrozen && inactive(bodies[i]) && inactive(bodies[j])) continue;   // already frozen above
            ContactManifold m;
            if (collide(bodies[i], bodies[j], m)) {
                m.indexA = i; m.indexB = j;
                if (!emit(m)) return StepStatus::Contacts;
            }
        }
        // Restore the stable (indexA, indexB) solve order the plain broadphase path
        // produced, so warm starting and solver ordering stay deterministic.
        if (sleeping)
            std::sort(nextManifolds_.begin(), nextManifolds_.end(),
                [](const ContactManifold& a, const ContactManifold& b) {
                    return a.indexA != b.indexA ? a.indexA < b.indexA : a.indexB < b.indexB;
                });
        if (profiling) timings.narrowphase = std::chrono::duration<real,std::milli>(std::chrono::steady_clock::now()-narrowStart).count();
        return StepStatus::Complete;
    }

    void correctPositions() {
        // Nonlinear Gauss-Seidel: recompute geometry after each pose update.
        // C=-depth; J*M^-1*J^T=K. A positive lambda reduces overlap by
        // delta xA=-invMassA*n*lambda, delta thetaA=-invIA*(rA x n)*lambda.
        // Velocities are untouched: no Baumgarte kinetic-energy injection.
        for(int iteration=0;iteration<solver.config().positionIterations;++iteration) {
            for (const auto& joint : joints) joint.correctPosition(bodies);
            for (const auto& old:manifolds_) {
                auto& A=bodies[old.indexA];auto& B=bodies[old.indexB];
                // Sleeping pairs need no position repair, and re-running
                // narrowphase for them is the single most expensive part of a
                // settled scene.
                if (!rowIsLive(A, B)) continue;
                ContactManifold current;
                if(!collide(A,B,current))continue;
                for(int p=0;p<current.count;++p) {
                    const auto& c=current.contactPoints[p];const Vec2 n=current.normal;
                    const Vec2 ra=c.position-A.position,rb=c.position-B.position;
                    const real ca=cross(ra,n),cb=cross(rb,n);
                    const real k=A.invMass+B.invMass+A.invInertia*ca*ca+B.invInertia*cb*cb;
                    if(k<=0)continue;
                    const real correction=solver.config().positionBeta
                        *clamp(c.depth-solver.config().slop,0,0.2)/k;
                    if(!A.isStatic()){A.translate(-n*(A.invMass*correction));
                        A.setAngle(A.angle-A.invInertia*ca*correction);}
                    if(!B.isStatic()){B.translate(n*(B.invMass*correction));
                        B.setAngle(B.angle+B.invInertia*cb*correction);}
                }
            }
        }
    }
public:
    // Pointer-free previous contacts: geometry and impulses have one owner.
    struct CachedManifold {
        std::array<ContactPoint, 2> contactPoints;
        Vec2 normal;
        real penetration = 0, friction = 0, restitution = 0;
        std::size_t indexA = 0, indexB = 0;
        int count = 0;
    };
    struct Snapshot {
        Vec2 position, velocity;
        real angle, angularVelocity, invMass, invInertia, radius;
        std::size_t vertexOffset, vertexCount;
        ShapeType type;
    };
private:
    // Public body-vector mutation cannot silently reuse impulses belonging to
    // a different body. Compare last post-step state/shape; invalidate globally
    // on external edits. Forces are deliberately excluded (normal inputs).
    bool cacheChanged() const {
        bool changed = snapshots_.size() != bodies.size();
        for (std::size_t i = 0; !changed && i < bodies.size(); ++i) {
            const auto& s = snapshots_[i]; const auto& b = bodies[i];
            // A sleeping body cannot have been edited without waking it first.
            if (!b.awake && s.invMass == b.invMass) continue;
            changed = s.position.x != b.position.x || s.position.y != b.position.y
                || s.velocity.x != b.velocity.x || s.velocity.y != b.velocity.y
                || s.angle != b.angle || s.angularVelocity != b.angularVelocity
                || s.invMass != b.invMass || s.invInertia != b.invInertia
                || s.type != b.shape.type || s.radius != b.shape.radius
                || s.vertexCount != b.shape.verts.size();
            for (std::size_t j = 0; !changed && j < b.shape.verts.size(); ++j)
                changed = vertices_[s.vertexOffset + j].x != b.shape.verts[j].x
                    || vertices_[s.vertexOffset + j].y != b.shape.verts[j].y;
        }
        return changed;
    }
    void restoreImpulses(real dt) {
        if (!solver.config().warmStarting) return;
        const real ratio = previousDt_ > 0 ? dt / previousDt_ : 1;
        std::size_t cached = 0;
        for (auto& m : manifolds_) {
            const BodyPair pair{m.indexA, m.indexB};
            while (cached < cache_.size()
                && BodyPair{cache_[cached].indexA, cache_[cached].indexB} < pair) ++cached;
            if (cached == cache_.size()) break;
            const auto& previous = cache_[cached];
            if (previous.indexA != m.indexA || previous.indexB != m.indexB
                || dot(previous.normal, m.normal) < 0.95) continue;
            for (int i = 0; i < m.count; ++i)
                for (int j = 0; j < previous.count; ++j) {
                    const auto& p = previous.contactPoints[j];
                    if (m.contactPoints[i].featureId == p.featureId) {
                        m.contactPoints[i].accN = p.accN * ratio;
                        m.contactPoints[i].accT = p.accT * ratio;
                    }
                }
        }
    }
    void saveImpulses(real dt) {
        cache_.clear(); // retain only contacts present this frame; no stale resurrection
        for (const auto& m : manifolds_) {
            cache_.push_back({});
            auto& entry = cache_.back();
            entry.indexA = m.indexA; entry.indexB = m.indexB;
            entry.normal = m.normal;
            entry.contactPoints = m.contactPoints;
            entry.penetration = m.penetration;
            entry.friction = m.friction;
            entry.restitution = m.restitution;
            entry.count = m.count;
        }
        snapshots_.resize(bodies.size());
        vertices_.clear();
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const auto& b = bodies[i];
            Snapshot& s = snapshots_[i];
            s.position = b.position; s.velocity = b.velocity;
            s.angle = b.angle; s.angularVelocity = b.angularVelocity;
            s.invMass = b.invMass; s.invInertia = b.invInertia;
            s.type = b.shape.type; s.radius = b.shape.radius;
            s.vertexOffset = vertices_.size(); s.vertexCount = b.shape.verts.size();
            vertices_.insert(vertices_.end(), b.shape.verts.begin(), b.shape.verts.end());
        }
        previousDt_ = dt;
    }
    Capacity capacity_;
    bool prepared_ = false;
    std::vector<CachedManifold> cache_;
    std::vector<Vec2> vertices_;
    std::vector<char> ccdScratch_;
    std::vector<Snapshot> snapshots_;
    real previousDt_ = 0; // duration represented by cached impulses (tick for PGS, slice for TGS)
    SolverMode previousMode_ = SolverMode::PGS;
    std::vector<TemporalContact> temporal_;
    RowSet rows_;
    Islands islands_;
    Broadphase broadphase_;
    std::vector<ContactManifold> manifolds_, nextManifolds_;
};

} // namespace phys
