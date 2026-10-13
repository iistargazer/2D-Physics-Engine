#pragma once
#include "DynamicTree.hpp"
#include <limits>

namespace phys {

using BodyPair = std::pair<std::size_t, std::size_t>;
enum class BroadphaseMode { DynamicTree, AllPairs };

class Broadphase {
public:
    void reserve(std::size_t bodies, std::size_t pairs) {
        tree_.reserve(bodies);
        proxies_.reserve(bodies); tight_.reserve(bodies);
        scratch_.reserve(bodies ? 2 * bodies - 1 : 0);
        pairs_.reserve(pairs);
    }
    // Full synchronization deliberately supports public World::bodies edits,
    // including teleport, rotation, erase/reorder, and replacement. Shrinking
    // rebuilds indices; equal-size edits simply refresh each indexed proxy.
    const std::vector<BodyPair>& findPairs(const std::vector<RigidBody>& bodies,
                                         BroadphaseMode mode = BroadphaseMode::DynamicTree,
                                         std::size_t pairLimit = std::numeric_limits<std::size_t>::max(),
                                         bool refreshSleeping = false) {
        pairs_.clear();
        exhausted_ = false;
        const auto emit = [&](std::size_t i, std::size_t j) {
            if (pairs_.size() == pairLimit) exhausted_ = true;
            else pairs_.emplace_back(i, j);
        };
        if (mode == BroadphaseMode::AllPairs) {
            for (std::size_t i = 0; i < bodies.size(); ++i)
                for (std::size_t j = i + 1; j < bodies.size(); ++j)
                    if (!(bodies[i].isStatic() && bodies[j].isStatic())) emit(i, j);
            return pairs_;
        }
        reinserts_ = 0;
        if (proxies_.size() > bodies.size()) { tree_.clear(); proxies_.clear(); }
        // Any change in body count invalidates the stored bounds wholesale.
        const bool resized = tight_.size() != bodies.size();
        tight_.resize(bodies.size());
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            // A sleeping body cannot have moved, so its stored tight bound is
            // still exact and its proxy needs no update. On a settled scene this
            // removes most of the broadphase cost.
            const bool asleep = !refreshSleeping && !resized && !bodies[i].awake && !bodies[i].isStatic();
            if (!asleep) tight_[i] = computeAABB(bodies[i]);
            if (i == proxies_.size()) proxies_.push_back(tree_.createProxy(tight_[i], i));
            else if (!asleep && tree_.moveProxy(proxies_[i], tight_[i])) ++reinserts_;
        }
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            tree_.query(tight_[i], [&](std::size_t j) {
                if (j > i && !(bodies[i].isStatic() && bodies[j].isStatic())
                    && tight_[i].overlaps(tight_[j])) emit(i, j);
            }, scratch_);
        }
        // Same lexicographic ordering as the old nested loops: physics should
        // not change merely because rotations alter tree traversal order.
        std::sort(pairs_.begin(), pairs_.end());
        return pairs_;
    }
    bool exhausted() const { return exhausted_; }
    const DynamicTree& tree() const { return tree_; }
    std::size_t reinserts() const { return reinserts_; }

private:
    DynamicTree tree_;
    std::vector<int> proxies_, scratch_;
    std::vector<AABB> tight_;
    std::vector<BodyPair> pairs_;
    std::size_t reinserts_ = 0;
    bool exhausted_ = false;
};

} // namespace phys
