#include <Moss/Physics/Physics2D/BroadPhase2D.hpp>
#include <Moss/Physics/Physics2D/BatchMath2D.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

DynamicAABBTree2D::DynamicAABBTree2D(float fat, float multiplier) : fatExtension_(std::max(0.0f, fat)), displacementMultiplier_(std::max(0.0f, multiplier)) {}
DynamicAABBTree2D::~DynamicAABBTree2D() = default;
DynamicAABBTree2D::DynamicAABBTree2D(DynamicAABBTree2D&&) noexcept = default;
DynamicAABBTree2D& DynamicAABBTree2D::operator=(DynamicAABBTree2D&&) noexcept = default;

AABB2 DynamicAABBTree2D::Combine(const AABB2& a, const AABB2& b) { return {Min(a.min, b.min), Max(a.max, b.max)}; }
float DynamicAABBTree2D::Perimeter(const AABB2& a) {
    Vec2 d = a.max - a.min;
    return 2.0f * (d.x + d.y);
}
int32_t DynamicAABBTree2D::AllocateNode() {
    if (freeList_ != -1) {
        int32_t node = freeList_;
        freeList_ = nodes_[size_t(node)].next;
        nodes_[size_t(node)] = Node{};
        nodes_[size_t(node)].height = 0;
        return node;
    }
    nodes_.push_back(Node{});
    nodes_.back().height = 0;
    return int32_t(nodes_.size() - 1);
}
void DynamicAABBTree2D::FreeNode(int32_t node) {
    Node& n = nodes_[size_t(node)];
    n.height = -1;
    n.parent = n.left = n.right = -1;
    n.next = freeList_;
    freeList_ = node;
}

BroadPhaseProxy2D DynamicAABBTree2D::CreateProxy(const AABB2& bounds, uint32_t data, BroadPhaseLayer2D layer) {
    if (!bounds.IsValid() || !std::isfinite(bounds.min.x) || !std::isfinite(bounds.min.y) ||
        !std::isfinite(bounds.max.x) || !std::isfinite(bounds.max.y)) return InvalidBroadPhaseProxy2D;
    int32_t id = AllocateNode();
    Node& node = nodes_[size_t(id)];
    node.bounds = bounds; node.bounds.Expand(fatExtension_);
    node.userData = data; node.layer = layer; node.height = 0;
    InsertLeaf(id); ++proxyCount_;
    return id;
}
bool DynamicAABBTree2D::DestroyProxy(BroadPhaseProxy2D proxy) {
    if (!IsValidProxy(proxy)) return false;
    RemoveLeaf(proxy); FreeNode(proxy); --proxyCount_; return true;
}
bool DynamicAABBTree2D::MoveProxy(BroadPhaseProxy2D proxy, const AABB2& bounds, Vec2 displacement) {
    if (!IsValidProxy(proxy) || !bounds.IsValid() ||
        !std::isfinite(bounds.min.x) || !std::isfinite(bounds.min.y) ||
        !std::isfinite(bounds.max.x) || !std::isfinite(bounds.max.y) ||
        !std::isfinite(displacement.x) || !std::isfinite(displacement.y)) return false;
    Node& node = nodes_[size_t(proxy)];
    if (node.bounds.Contains(bounds.min) && node.bounds.Contains(bounds.max)) return false;
    RemoveLeaf(proxy);
    AABB2 fat = bounds; fat.Expand(fatExtension_);
    Vec2 d = displacement * displacementMultiplier_;
    if (d.x < 0) fat.min.x += d.x; else fat.max.x += d.x;
    if (d.y < 0) fat.min.y += d.y; else fat.max.y += d.y;
    node.bounds = fat;
    InsertLeaf(proxy);
    return true;
}
bool DynamicAABBTree2D::UpdateProxyLayer(BroadPhaseProxy2D proxy, BroadPhaseLayer2D layer) {
    if (!IsValidProxy(proxy)) return false;
    nodes_[size_t(proxy)].layer = layer; return true;
}
bool DynamicAABBTree2D::IsValidProxy(BroadPhaseProxy2D proxy) const {
    return proxy >= 0 && size_t(proxy) < nodes_.size() && nodes_[size_t(proxy)].height >= 0 && nodes_[size_t(proxy)].IsLeaf();
}
uint32_t DynamicAABBTree2D::GetUserData(BroadPhaseProxy2D proxy) const {
    return IsValidProxy(proxy) ? nodes_[size_t(proxy)].userData : 0xffffffffu;
}
BroadPhaseLayer2D DynamicAABBTree2D::GetLayer(BroadPhaseProxy2D proxy) const {
    return IsValidProxy(proxy) ? nodes_[size_t(proxy)].layer : BroadPhaseLayer2D::QueryOnly;
}
AABB2 DynamicAABBTree2D::GetFatBounds(BroadPhaseProxy2D proxy) const {
    return IsValidProxy(proxy) ? nodes_[size_t(proxy)].bounds : AABB2{};
}
void DynamicAABBTree2D::InsertLeaf(int32_t leaf) {
    if (root_ == -1) { root_ = leaf; nodes_[size_t(root_)].parent = -1; return; }
    const AABB2 leafBounds = nodes_[size_t(leaf)].bounds;
    int32_t sibling = root_;
    while (!nodes_[size_t(sibling)].IsLeaf()) {
        const Node& n = nodes_[size_t(sibling)];
        const float area = Perimeter(n.bounds);
        const float combined = Perimeter(Combine(n.bounds, leafBounds));
        const float inheritance = 2.0f * (combined - area);
        const int32_t left = n.left, right = n.right;
        const float leftCost = (nodes_[size_t(left)].IsLeaf()
            ? Perimeter(Combine(leafBounds, nodes_[size_t(left)].bounds)) :
              Perimeter(Combine(leafBounds, nodes_[size_t(left)].bounds)) - Perimeter(nodes_[size_t(left)].bounds)) + inheritance;
        const float rightCost = (nodes_[size_t(right)].IsLeaf()
            ? Perimeter(Combine(leafBounds, nodes_[size_t(right)].bounds)) :
              Perimeter(Combine(leafBounds, nodes_[size_t(right)].bounds)) - Perimeter(nodes_[size_t(right)].bounds)) + inheritance;
        const float parentCost = 2.0f * combined;
        if (parentCost < leftCost && parentCost < rightCost) break;
        sibling = leftCost < rightCost ? left : right;
    }
    const int32_t oldParent = nodes_[size_t(sibling)].parent;
    const int32_t newParent = AllocateNode();
    Node& p = nodes_[size_t(newParent)];
    p.parent = oldParent; p.bounds = Combine(leafBounds, nodes_[size_t(sibling)].bounds);
    p.height = nodes_[size_t(sibling)].height + 1; p.left = sibling; p.right = leaf;
    nodes_[size_t(sibling)].parent = newParent; nodes_[size_t(leaf)].parent = newParent;
    if (oldParent == -1) root_ = newParent;
    else if (nodes_[size_t(oldParent)].left == sibling) nodes_[size_t(oldParent)].left = newParent;
    else nodes_[size_t(oldParent)].right = newParent;
    RefitUpward(newParent);
}
void DynamicAABBTree2D::RemoveLeaf(int32_t leaf) {
    if (leaf == root_) { root_ = -1; return; }
    const int32_t parent = nodes_[size_t(leaf)].parent;
    const int32_t grand = nodes_[size_t(parent)].parent;
    const int32_t sibling = nodes_[size_t(parent)].left == leaf ? nodes_[size_t(parent)].right : nodes_[size_t(parent)].left;
    if (grand != -1) {
        if (nodes_[size_t(grand)].left == parent) nodes_[size_t(grand)].left = sibling;
        else nodes_[size_t(grand)].right = sibling;
        nodes_[size_t(sibling)].parent = grand;
        FreeNode(parent);
        RefitUpward(grand);
    } else {
        root_ = sibling; nodes_[size_t(sibling)].parent = -1; FreeNode(parent);
    }
    nodes_[size_t(leaf)].parent = -1;
}
int32_t DynamicAABBTree2D::Balance(int32_t iA) {
    Node& A = nodes_[size_t(iA)];
    if (A.IsLeaf() || A.height < 2) return iA;
    const int32_t iB = A.left, iC = A.right;
    Node& B = nodes_[size_t(iB)]; Node& C = nodes_[size_t(iC)];
    const int balance = C.height - B.height;
    if (balance > 1) {
        const int32_t iF = C.left, iG = C.right;
        Node& F = nodes_[size_t(iF)]; Node& G = nodes_[size_t(iG)];
        C.left = iA; C.parent = A.parent; A.parent = iC;
        if (C.parent != -1) { if (nodes_[size_t(C.parent)].left == iA) nodes_[size_t(C.parent)].left = iC; else nodes_[size_t(C.parent)].right = iC; }
        else root_ = iC;
        if (F.height > G.height) {
            C.right = iF; A.right = iG; G.parent = iA; F.parent = iC;
            A.bounds = Combine(B.bounds, G.bounds); C.bounds = Combine(A.bounds, F.bounds);
            A.height = 1 + std::max(B.height, G.height); C.height = 1 + std::max(A.height, F.height);
        } else {
            C.right = iG; A.right = iF; F.parent = iA; G.parent = iC;
            A.bounds = Combine(B.bounds, F.bounds); C.bounds = Combine(A.bounds, G.bounds);
            A.height = 1 + std::max(B.height, F.height); C.height = 1 + std::max(A.height, G.height);
        }
        return iC;
    }
    if (balance < -1) {
        const int32_t iD = B.left, iE = B.right;
        Node& D = nodes_[size_t(iD)]; Node& E = nodes_[size_t(iE)];
        B.left = iA; B.parent = A.parent; A.parent = iB;
        if (B.parent != -1) { if (nodes_[size_t(B.parent)].left == iA) nodes_[size_t(B.parent)].left = iB; else nodes_[size_t(B.parent)].right = iB; }
        else root_ = iB;
        if (D.height > E.height) {
            B.right = iD; A.left = iE; E.parent = iA; D.parent = iB;
            A.bounds = Combine(C.bounds, E.bounds); B.bounds = Combine(A.bounds, D.bounds);
            A.height = 1 + std::max(C.height, E.height); B.height = 1 + std::max(A.height, D.height);
        } else {
            B.right = iE; A.left = iD; D.parent = iA; E.parent = iB;
            A.bounds = Combine(C.bounds, D.bounds); B.bounds = Combine(A.bounds, E.bounds);
            A.height = 1 + std::max(C.height, D.height); B.height = 1 + std::max(A.height, E.height);
        }
        return iB;
    }
    return iA;
}
void DynamicAABBTree2D::RefitUpward(int32_t node) {
    while (node != -1) {
        node = Balance(node);
        Node& n = nodes_[size_t(node)];
        if (!n.IsLeaf()) {
            const Node& l = nodes_[size_t(n.left)]; const Node& r = nodes_[size_t(n.right)];
            n.height = 1 + std::max(l.height, r.height);
            n.bounds = Combine(l.bounds, r.bounds);
        }
        node = n.parent;
    }
}
bool DynamicAABBTree2D::LayerAllowed(BroadPhaseLayer2D layer, uint8_t mask) const {
    return (mask & (uint8_t(1u) << uint8_t(layer))) != 0;
}
void DynamicAABBTree2D::Query(const AABB2& bounds, const std::function<bool(uint32_t)>& callback, uint8_t mask) const {
    if (root_ == -1 || !bounds.IsValid() || !callback ||
        !std::isfinite(bounds.min.x) || !std::isfinite(bounds.min.y) ||
        !std::isfinite(bounds.max.x) || !std::isfinite(bounds.max.y)) return;
    std::vector<int32_t> stack; stack.reserve(64); stack.push_back(root_);
    // Test four independent node bounds together. The tree traversal remains
    // scalar, while the expensive overlap predicates are evaluated in SIMD lanes.
    while (!stack.empty()) {
        int32_t ids[4]{};float minX[4]{},minY[4]{},maxX[4]{},maxY[4]{};uint32_t count=0;
        while(count<4&&!stack.empty()){
            const int32_t id=stack.back();stack.pop_back();ids[count]=id;
            const AABB2& b=nodes_[size_t(id)].bounds;
            minX[count]=b.min.x;minY[count]=b.min.y;maxX[count]=b.max.x;maxY[count]=b.max.y;++count;
        }
        uint32_t overlapMask=0;
        if(count==4)overlapMask=BatchAABBOverlap4(minX,minY,maxX,maxY,bounds);
        else for(uint32_t i=0;i<count;++i)if(minX[i]<=bounds.max.x&&maxX[i]>=bounds.min.x&&minY[i]<=bounds.max.y&&maxY[i]>=bounds.min.y)overlapMask|=1u<<i;
        for(uint32_t i=0;i<count;++i){
            if((overlapMask&(1u<<i))==0)continue;
            const Node& n=nodes_[size_t(ids[i])];
            if(n.IsLeaf()){if(LayerAllowed(n.layer,mask)&&!callback(n.userData))return;}
            else {stack.push_back(n.left);stack.push_back(n.right);}
        }
    }
}
bool DynamicAABBTree2D::SegmentIntersectsAABB(Vec2 a, Vec2 b, const AABB2& box) {
    Vec2 d = b - a; float lo = 0.0f, hi = 1.0f;
    for (int axis = 0; axis < 2; ++axis) {
        float p = axis == 0 ? a.x : a.y, v = axis == 0 ? d.x : d.y;
        float mn = axis == 0 ? box.min.x : box.min.y, mx = axis == 0 ? box.max.x : box.max.y;
        if (std::abs(v) < kEpsilon) { if (p < mn || p > mx) return false; }
        else { float t1 = (mn - p) / v, t2 = (mx - p) / v; if (t1 > t2) std::swap(t1, t2); lo = std::max(lo, t1); hi = std::min(hi, t2); if (lo > hi) return false; }
    }
    return true;
}
void DynamicAABBTree2D::RayQuery(Vec2 from, Vec2 to, const std::function<bool(uint32_t)>& callback, uint8_t mask) const {
    if (root_ == -1 || !callback || !std::isfinite(from.x) || !std::isfinite(from.y) ||
        !std::isfinite(to.x) || !std::isfinite(to.y)) return;
    std::vector<int32_t> stack; stack.reserve(64); stack.push_back(root_);
    while (!stack.empty()) {
        int32_t id = stack.back(); stack.pop_back(); const Node& n = nodes_[size_t(id)];
        if (!SegmentIntersectsAABB(from, to, n.bounds)) continue;
        if (n.IsLeaf()) { if (LayerAllowed(n.layer, mask) && !callback(n.userData)) return; }
        else { stack.push_back(n.left); stack.push_back(n.right); }
    }
}
std::vector<uint32_t> DynamicAABBTree2D::Query(const AABB2& bounds, uint8_t mask) const {
    std::vector<uint32_t> out; Query(bounds, [&](uint32_t id) { out.push_back(id); return true; }, mask); return out;
}
void DynamicAABBTree2D::Rebuild() {
    // Preserve leaf indices: they are externally held proxy handles.
    std::vector<std::pair<int32_t, Node>> leaves; leaves.reserve(proxyCount_);
    const int32_t oldSize = int32_t(nodes_.size());
    std::vector<uint8_t> isLeaf(size_t(oldSize),0);
    for (int32_t i = 0; i < oldSize; ++i) {
        const Node& n = nodes_[size_t(i)];
        if (n.height == 0 && n.IsLeaf()) { leaves.emplace_back(i, n); isLeaf[size_t(i)]=1; }
    }
    nodes_.assign(size_t(oldSize), Node{});
    root_ = freeList_ = -1; proxyCount_ = 0;
    // Seed the allocator with old internal/free slots before reinserting leaves.
    // This keeps the rebuild from growing nodes_ by O(proxyCount) each time.
    for (int32_t i = 0; i < oldSize; ++i) {
        if (!isLeaf[size_t(i)]) {
            nodes_[size_t(i)].height = -1;
            nodes_[size_t(i)].next = freeList_;
            freeList_ = i;
        }
    }
    for (const auto& item : leaves) {
        const int32_t id = item.first;
        nodes_[size_t(id)] = item.second;
        nodes_[size_t(id)].parent = -1;
        nodes_[size_t(id)].left = nodes_[size_t(id)].right = -1;
        nodes_[size_t(id)].height = 0;
        InsertLeaf(id); ++proxyCount_;
    }
}
void DynamicAABBTree2D::Clear() { nodes_.clear(); root_ = freeList_ = -1; proxyCount_ = 0; }

bool DynamicAABBTree2D::Validate() const {
    std::vector<uint8_t> visited(nodes_.size(),0);
    uint32_t leaves = 0;
    std::function<bool(int32_t)> visit = [&](int32_t id) {
        if (id < 0 || size_t(id) >= nodes_.size() || visited[size_t(id)] != 0) return false;
        visited[size_t(id)] = 1;
        const Node& n = nodes_[size_t(id)];
        if (n.height < 0) return false;
        if (n.IsLeaf()) { ++leaves; return n.height == 0 && n.right == -1; }
        if (n.left < 0 || n.right < 0 || size_t(n.left)>=nodes_.size() || size_t(n.right)>=nodes_.size()) return false;
        const Node& l = nodes_[size_t(n.left)]; const Node& r = nodes_[size_t(n.right)];
        if (l.parent != id || r.parent != id || l.height < 0 || r.height < 0) return false;
        const AABB2 combined = Combine(l.bounds, r.bounds);
        if (n.height != 1 + std::max(l.height, r.height)) return false;
        if (!n.bounds.Contains(combined.min) || !n.bounds.Contains(combined.max)) return false;
        return visit(n.left) && visit(n.right);
    };
    if (root_ == -1) { if (proxyCount_ != 0) return false; }
    else if (size_t(root_)>=nodes_.size() || nodes_[size_t(root_)].parent!=-1 || !visit(root_)) return false;
    if (leaves != proxyCount_) return false;

    // Every unused node must occur exactly once in the free list; every active
    // node must be reachable from the root. This also catches free-list cycles.
    int32_t free = freeList_;
    while (free != -1) {
        if (free < 0 || size_t(free)>=nodes_.size() || visited[size_t(free)]!=0) return false;
        const Node& n=nodes_[size_t(free)];
        if (n.height>=0) return false;
        visited[size_t(free)]=2;
        free=n.next;
    }
    for (size_t i=0;i<nodes_.size();++i) {
        if (nodes_[i].height>=0 && visited[i]!=1) return false;
        if (nodes_[i].height<0 && visited[i]!=2) return false;
    }
    return true;
}
BroadPhaseTreeStats2D DynamicAABBTree2D::GetStats() const {
    BroadPhaseTreeStats2D s; s.proxyCount = proxyCount_;
    if (root_ == -1) return s;
    s.height = uint32_t(std::max(0, nodes_[size_t(root_)].height));
    float total = 0.0f; uint32_t used = 0;
    for (const Node& n : nodes_) if (n.height >= 0) { ++used; total += Perimeter(n.bounds); if (!n.IsLeaf()) s.maximumBalance = std::max(s.maximumBalance, uint32_t(std::abs(nodes_[size_t(n.left)].height - nodes_[size_t(n.right)].height))); }
    s.nodeCount = used; float rootArea = Perimeter(nodes_[size_t(root_)].bounds); s.areaRatio = rootArea > kEpsilon ? total / rootArea : 0.0f; return s;
}