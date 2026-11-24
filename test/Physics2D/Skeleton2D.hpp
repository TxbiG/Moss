#pragma once
#include "Shapes2D.hpp"
#include <string>
#include <vector>

struct MOSS_API SkeletonBoneDef2D {
    std::string name;
    int parent = -1;              // parent must appear before child
    Transform2D bindLocal{};
};
struct MOSS_API SkeletonBone2D {
    std::string name;
    int parent = -1;
    Transform2D local{};
    Transform2D world{};
};
struct MOSS_API SkeletonPose2D {
    std::vector<Transform2D> localTransforms;
};

class MOSS_API Skeleton2D {
public:
    bool Build(const std::vector<SkeletonBoneDef2D>& defs) {
        bones_.clear(); bones_.reserve(defs.size());
        for (size_t i=0;i<defs.size();++i) {
            if (defs[i].parent>=int(i) || defs[i].parent < -1) { bones_.clear(); return false; }
            bones_.push_back({defs[i].name,defs[i].parent,defs[i].bindLocal,defs[i].bindLocal});
        }
        UpdateWorldTransforms(); return !bones_.empty();
    }
    size_t GetBoneCount() const { return bones_.size(); }
    const SkeletonBone2D* GetBone(size_t i) const { return i<bones_.size()?&bones_[i]:nullptr; }
    SkeletonBone2D* GetBone(size_t i) { return i<bones_.size()?&bones_[i]:nullptr; }
    int FindBone(const std::string& name) const {
        for (size_t i=0;i<bones_.size();++i) if (bones_[i].name==name) return int(i);
        return -1;
    }
    bool SetLocalTransform(size_t i,const Transform2D& t) { if(i>=bones_.size())return false;bones_[i].local=t;return true; }
    bool SetLocalPosition(size_t i,Vec2 p) { if(i>=bones_.size())return false;bones_[i].local.position=p;return true; }
    bool SetLocalAngle(size_t i,float a) { if(i>=bones_.size())return false;bones_[i].local.angle=WrapAngle(a);return true; }
    void ResetToBindPose(const std::vector<SkeletonBoneDef2D>& defs) {
        if(defs.size()!=bones_.size()) return;
        for(size_t i=0;i<defs.size();++i) bones_[i].local=defs[i].bindLocal;
        UpdateWorldTransforms();
    }
    void UpdateWorldTransforms() {
        for(size_t i=0;i<bones_.size();++i) {
            auto& b=bones_[i];
            if(b.parent<0) b.world=b.local;
            else { const auto& p=bones_[size_t(b.parent)].world;b.world.position=p.Apply(b.local.position);b.world.angle=WrapAngle(p.angle+b.local.angle); }
        }
    }
    SkeletonPose2D CapturePose() const { SkeletonPose2D pose;pose.localTransforms.reserve(bones_.size());for(const auto& b:bones_)pose.localTransforms.push_back(b.local);return pose; }
    bool ApplyPose(const SkeletonPose2D& pose,float weight=1.0f) {
        if(pose.localTransforms.size()!=bones_.size()) return false;
        weight=Clamp(weight,0.0f,1.0f);
        for(size_t i=0;i<bones_.size();++i) {
            auto& dst=bones_[i].local; const auto& src=pose.localTransforms[i];
            dst.position=dst.position*(1.0f-weight)+src.position*weight;
            dst.angle=WrapAngle(dst.angle+WrapAngle(src.angle-dst.angle)*weight);
        }
        UpdateWorldTransforms(); return true;
    }
private:
    std::vector<SkeletonBone2D> bones_;
};
