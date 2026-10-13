#pragma once
#include "AABB.hpp"
#include <cassert>
#include <utility>

namespace phys {

class DynamicTree {
public:
    struct Node {
        AABB bounds;
        int parent = -1, left = -1, right = -1, height = -1, next = -1;
        std::size_t body = 0;
        bool leaf() const { return left == -1; }
    };

    explicit DynamicTree(real margin = 0.1) : margin_(margin) { assert(margin >= 0); }
    void reserve(std::size_t bodies) { nodes_.reserve(bodies ? 2 * bodies - 1 : 0); }
    int createProxy(const AABB& tight, std::size_t body) {
        const int id = allocate();
        nodes_[id].bounds = tight.fatten(margin_);
        nodes_[id].body = body;
        nodes_[id].height = 0;
        insert(id);
        return id;
    }
    void destroyProxy(int id) { remove(id); release(id); }
    bool moveProxy(int id, const AABB& tight) {
        if (nodes_[id].bounds.contains(tight)) return false;
        remove(id);
        nodes_[id].bounds = tight.fatten(margin_);
        insert(id);
        return true;
    }
    void clear() { nodes_.clear(); root_ = free_ = -1; }
    int height() const { return root_ == -1 ? 0 : nodes_[root_].height; }
    const Node& node(int id) const { return nodes_[id]; }
    const std::vector<Node>& nodes() const { return nodes_; }

    // Caller supplies reusable scratch to avoid a heap allocation per query.
    template<class Callback>
    void query(const AABB& bounds, Callback callback, std::vector<int>& scratch) const {
        scratch.clear();
        if (root_ != -1) scratch.push_back(root_);
        while (!scratch.empty()) {
            const int id = scratch.back();
            scratch.pop_back();
            const Node& n = nodes_[id];
            if (!bounds.overlaps(n.bounds)) continue;
            if (n.leaf()) callback(n.body);
            else { scratch.push_back(n.left); scratch.push_back(n.right); }
        }
    }
    bool validate() const {
        if (root_ == -1) {
            for (const auto& n : nodes_) if (n.height >= 0) return false;
            return true;
        }
        std::vector<bool> seen(nodes_.size(), false);
        if (!validateNode(root_, -1, seen)) return false;
        for (std::size_t i = 0; i < nodes_.size(); ++i)
            if ((nodes_[i].height >= 0) != seen[i]) return false;
        return true;
    }

private:
    int allocate() {
        if (free_ == -1) {
            nodes_.push_back({});
            return static_cast<int>(nodes_.size() - 1);
        }
        const int id = free_;
        free_ = nodes_[id].next;
        nodes_[id] = {};
        return id;
    }
    void release(int id) { nodes_[id] = {}; nodes_[id].next = free_; free_ = id; }
    void refit(int id) {
        Node& n = nodes_[id];
        n.bounds = AABB::unite(nodes_[n.left].bounds, nodes_[n.right].bounds);
        n.height = 1 + std::max(nodes_[n.left].height, nodes_[n.right].height);
    }
    void replaceChild(int parent, int oldChild, int newChild) {
        if (parent == -1) root_ = newChild;
        else if (nodes_[parent].left == oldChild) nodes_[parent].left = newChild;
        else nodes_[parent].right = newChild;
        if (newChild != -1) nodes_[newChild].parent = parent;
    }
    // If the right child is taller, promote it; attach its shorter grandchild
    // under the old root. The mirror operation handles a taller left child.
    // Leaf sets are unchanged; recomputing unions preserves conservativeness.
    int balance(int a) {
        if (nodes_[a].leaf() || nodes_[a].height < 2) return a;
        const int b = nodes_[a].left, c = nodes_[a].right;
        const int delta = nodes_[c].height - nodes_[b].height;
        if (delta > 1) {
            const int f = nodes_[c].left, g = nodes_[c].right;
            replaceChild(nodes_[a].parent, a, c);
            nodes_[c].left = a;
            nodes_[a].parent = c;
            const bool fTaller = nodes_[f].height > nodes_[g].height;
            nodes_[c].right = fTaller ? f : g;
            nodes_[a].right = fTaller ? g : f;
            nodes_[nodes_[c].right].parent = c;
            nodes_[nodes_[a].right].parent = a;
            refit(a); refit(c);
            return c;
        }
        if (delta < -1) {
            const int d = nodes_[b].left, e = nodes_[b].right;
            replaceChild(nodes_[a].parent, a, b);
            nodes_[b].right = a;
            nodes_[a].parent = b;
            const bool dTaller = nodes_[d].height > nodes_[e].height;
            nodes_[b].left = dTaller ? d : e;
            nodes_[a].left = dTaller ? e : d;
            nodes_[nodes_[b].left].parent = b;
            nodes_[nodes_[a].left].parent = a;
            refit(a); refit(b);
            return b;
        }
        return a;
    }
    void repair(int id) {
        while (id != -1) {
            id = balance(id);
            refit(id);
            id = nodes_[id].parent;
        }
    }
    void insert(int leaf) {
        if (root_ == -1) { root_ = leaf; nodes_[leaf].parent = -1; return; }
        const AABB bounds = nodes_[leaf].bounds;
        int sibling = root_;
        while (!nodes_[sibling].leaf()) {
            const Node& n = nodes_[sibling];
            const real combined = AABB::unite(n.bounds, bounds).perimeter();
            // Descend to a leaf (rather than splicing above a tall subtree)
            // so one local AVL rotation restores balance at each ancestor.
            // Enlarging the current bound contributes inherited = 2 delta P.
            const real inherited = 2 * (combined - n.bounds.perimeter());
            const auto descendCost = [&](int child) {
                const Node& ch = nodes_[child];
                const real p = AABB::unite(ch.bounds, bounds).perimeter();
                return inherited + (ch.leaf() ? p : p - ch.bounds.perimeter());
            };
            const real leftCost = descendCost(n.left), rightCost = descendCost(n.right);
            sibling = leftCost <= rightCost ? n.left : n.right;
        }
        const int oldParent = nodes_[sibling].parent;
        const int parent = allocate(); // may reallocate: never retain Node& here
        nodes_[parent].left = sibling;
        nodes_[parent].right = leaf;
        nodes_[parent].height = 1;
        replaceChild(oldParent, sibling, parent);
        nodes_[sibling].parent = nodes_[leaf].parent = parent;
        repair(parent);
    }
    void remove(int leaf) {
        if (leaf == root_) { root_ = -1; nodes_[leaf].parent = -1; return; }
        const int parent = nodes_[leaf].parent;
        const int grand = nodes_[parent].parent;
        const int sibling = nodes_[parent].left == leaf ? nodes_[parent].right : nodes_[parent].left;
        replaceChild(grand, parent, sibling);
        release(parent);
        nodes_[leaf].parent = -1;
        if (grand != -1) repair(grand);
    }
    bool validateNode(int id, int parent, std::vector<bool>& seen) const {
        if (id < 0 || static_cast<std::size_t>(id) >= nodes_.size() || seen[id]) return false;
        seen[id] = true;
        const Node& n = nodes_[id];
        if (n.parent != parent || n.height < 0) return false;
        if (n.leaf()) return n.right == -1 && n.height == 0;
        if (!validateNode(n.left, id, seen) || !validateNode(n.right, id, seen)) return false;
        const Node& l = nodes_[n.left]; const Node& r = nodes_[n.right];
        const AABB expected = AABB::unite(l.bounds, r.bounds);
        return n.height == 1 + std::max(l.height, r.height)
            && std::abs(l.height - r.height) <= 1
            && n.bounds.contains(expected) && expected.contains(n.bounds);
    }
    std::vector<Node> nodes_;
    int root_ = -1, free_ = -1;
    real margin_;
};

} // namespace phys
