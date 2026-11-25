#pragma once
#include "Shapes2D.hpp"
#include <cstdint>
#include <functional>
#include <vector>

using BroadPhaseProxy2D = int32_t;
constexpr BroadPhaseProxy2D InvalidBroadPhaseProxy2D = -1;

enum class BroadPhaseLayer2D : uint8_t { Static = 0, Moving = 1, QueryOnly = 2 };

struct BroadPhaseTreeStats2D {
    uint32_t proxyCount = 0;
    uint32_t nodeCount = 0;
    uint32_t height = 0;
    uint32_t maximumBalance = 0;
    float areaRatio = 0.0f;
};

// Incremental 2D dynamic AABB tree. Leaves store stable client IDs; internal nodes
// are binary and are rebalanced after insert/remove. Bounds are fattened so small
// movements do not force an update on every frame.
class DynamicAABBTree2D {
public:
    explicit DynamicAABBTree2D(float fatExtension = 0.1f, float displacementMultiplier = 2.0f);
    ~DynamicAABBTree2D();
    DynamicAABBTree2D(DynamicAABBTree2D&&) noexcept;
    DynamicAABBTree2D& operator=(DynamicAABBTree2D&&) noexcept;
    DynamicAABBTree2D(const DynamicAABBTree2D&) = delete;
    DynamicAABBTree2D& operator=(const DynamicAABBTree2D&) = delete;

    BroadPhaseProxy2D CreateProxy(const AABB2& bounds, uint32_t userData,
                                  BroadPhaseLayer2D layer = BroadPhaseLayer2D::Moving);
    bool DestroyProxy(BroadPhaseProxy2D proxy);
    bool MoveProxy(BroadPhaseProxy2D proxy, const AABB2& bounds, Vec2 displacement = {});
    bool UpdateProxyLayer(BroadPhaseProxy2D proxy, BroadPhaseLayer2D layer);
    bool IsValidProxy(BroadPhaseProxy2D proxy) const;
    uint32_t GetUserData(BroadPhaseProxy2D proxy) const;
    BroadPhaseLayer2D GetLayer(BroadPhaseProxy2D proxy) const;
    AABB2 GetFatBounds(BroadPhaseProxy2D proxy) const;

    void Query(const AABB2& bounds, const std::function<bool(uint32_t)>& callback,
               uint8_t layerMask = 0xff) const;
    void RayQuery(Vec2 from, Vec2 to, const std::function<bool(uint32_t)>& callback,
                  uint8_t layerMask = 0xff) const;
    std::vector<uint32_t> Query(const AABB2& bounds, uint8_t layerMask = 0xff) const;
    void Rebuild();
    void Clear();
    bool Validate() const;
    BroadPhaseTreeStats2D GetStats() const;

private:
    struct Node {
        AABB2 bounds{};
        int32_t parent = -1, left = -1, right = -1, next = -1;
        int32_t height = -1;
        uint32_t userData = 0;
        BroadPhaseLayer2D layer = BroadPhaseLayer2D::Moving;
        bool IsLeaf() const { return left == -1; }
    };
    int32_t AllocateNode();
    void FreeNode(int32_t node);
    void InsertLeaf(int32_t leaf);
    void RemoveLeaf(int32_t leaf);
    int32_t Balance(int32_t node);
    void RefitUpward(int32_t node);
    static AABB2 Combine(const AABB2& a, const AABB2& b);
    static float Perimeter(const AABB2& a);
    static bool SegmentIntersectsAABB(Vec2 a, Vec2 b, const AABB2& box);
    bool LayerAllowed(BroadPhaseLayer2D layer, uint8_t mask) const;

    std::vector<Node> nodes_;
    int32_t root_ = -1, freeList_ = -1;
    uint32_t proxyCount_ = 0;
    float fatExtension_ = 0.1f;
    float displacementMultiplier_ = 2.0f;
};